import subprocess
import sys
import importlib
import threading


def ensure_dependency(module_name, pip_name=None):
    try:
        importlib.import_module(module_name)
    except ImportError:
        print(f"{module_name} not found. Installing...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", pip_name or module_name])
        importlib.invalidate_caches()


ensure_dependency("PIL", "Pillow")

import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Gtk, Gio, GLib, Adw

from pathlib import Path
from PIL import Image

SOURCE_IMAGE_NAME = "replacement.png"

SUPPORTED_EXTENSIONS = {
    ".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tiff", ".tif", ".webp"
}

EXT_TO_FORMAT = {
    ".png": "PNG",
    ".jpg": "JPEG",
    ".jpeg": "JPEG",
    ".bmp": "BMP",
    ".gif": "GIF",
    ".tiff": "TIFF",
    ".tif": "TIFF",
    ".webp": "WEBP",
}

FORMATS_NEEDING_RGB = {"JPEG", "BMP"}






def find_image_files(target_folder: Path):
    for path in target_folder.rglob("*"):
        if path.is_file() and path.suffix.lower() in SUPPORTED_EXTENSIONS:
            yield path


def convert_and_save(source_img: "Image.Image", dest_path: Path):
    target_format = EXT_TO_FORMAT[dest_path.suffix.lower()]
    img = source_img

    if target_format in FORMATS_NEEDING_RGB and img.mode in ("RGBA", "P", "LA"):
        background = Image.new("RGB", img.size, (255, 255, 255))
        rgba = img.convert("RGBA")
        background.paste(rgba, mask=rgba.split()[-1])
        img = background
    elif target_format == "PNG" and img.mode == "P":
        img = img.convert("RGBA")
    elif target_format not in FORMATS_NEEDING_RGB and img.mode not in ("RGB", "RGBA", "P"):
        img = img.convert("RGBA")

    save_kwargs = {}
    if target_format == "JPEG":
        save_kwargs["quality"] = 95
    img.save(dest_path, format=target_format, **save_kwargs)


def run_replacement(target_folders, source_path: Path, log_line, is_cancelled):
    try:
        source_img = Image.open(source_path)
        source_img.load()
    except Exception as e:
        log_line(f"Failed to open source image '{source_path}': {e}\n")
        return

    all_image_files = []
    for folder in target_folders:
        found = list(find_image_files(folder))
        log_line(f"Found {len(found)} image file(s) under '{folder}'.\n")
        all_image_files.extend(found)

    if not all_image_files:
        log_line("No supported image files found in the selected folders.\n")
        return

    log_line(f"\nSource image: '{source_path}'\n")
    log_line(f"Total files to replace: {len(all_image_files)}\n\n")

    replaced = 0
    for dest_path in all_image_files:
        if is_cancelled():
            log_line("\nCancelled.\n")
            return
        if dest_path.resolve() == source_path.resolve():
            continue
        try:
            convert_and_save(source_img, dest_path)
            log_line(f"Replaced ({dest_path.suffix.lower()}): {dest_path}\n")
            replaced += 1
        except Exception as e:
            log_line(f"Failed to replace '{dest_path}': {e}\n")

    log_line(f"\nDone. Replaced {replaced} file(s).\n")






class ImageReplacerWindow(Adw.ApplicationWindow):
    def __init__(self, app):
        super().__init__(application=app, title="Image Replacer")
        self.set_default_size(720, 660)

        self.target_folders: list[Path] = []
        self.run_cancelled = False
        self.run_thread: threading.Thread | None = None

        toolbar_view = Adw.ToolbarView()
        header = Adw.HeaderBar()
        toolbar_view.add_top_bar(header)
        self.set_content(toolbar_view)

        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=16)
        root.set_margin_top(16)
        root.set_margin_bottom(16)
        root.set_margin_start(16)
        root.set_margin_end(16)
        toolbar_view.set_content(root)


        folders_label = Gtk.Label(label="Target folders (searched recursively)", xalign=0)
        folders_label.add_css_class("heading")
        root.append(folders_label)

        folders_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        root.append(folders_row)

        list_scroller = Gtk.ScrolledWindow(hexpand=True, vexpand=True, min_content_height=180)
        list_scroller.add_css_class("card")
        folders_row.append(list_scroller)

        self.folder_store = Gtk.StringList()
        self.folder_selection = Gtk.MultiSelection(model=self.folder_store)
        factory = Gtk.SignalListItemFactory()
        factory.connect("setup", self._setup_folder_row)
        factory.connect("bind", self._bind_folder_row)
        self.folder_listview = Gtk.ListView(model=self.folder_selection, factory=factory)
        list_scroller.set_child(self.folder_listview)

        btn_col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        folders_row.append(btn_col)

        add_btn = Gtk.Button(label="Add Folders…")
        add_btn.add_css_class("suggested-action")
        add_btn.connect("clicked", self.on_add_folders_clicked)
        btn_col.append(add_btn)

        paste_btn = Gtk.Button(label="Paste Paths…")
        paste_btn.connect("clicked", self.on_paste_paths_clicked)
        btn_col.append(paste_btn)

        remove_btn = Gtk.Button(label="Remove Selected")
        remove_btn.connect("clicked", self.on_remove_selected_clicked)
        btn_col.append(remove_btn)

        clear_btn = Gtk.Button(label="Clear All")
        clear_btn.connect("clicked", self.on_clear_folders_clicked)
        btn_col.append(clear_btn)

        hint = Gtk.Label(
            label="Add Folders… opens the system picker with multi-select enabled — "
                  "Ctrl/Shift-click as many folders as you like, then hit Open once.",
            xalign=0, wrap=True
        )
        hint.add_css_class("dim-label")
        hint.add_css_class("caption")
        root.append(hint)


        source_label = Gtk.Label(label="Source image (any format)", xalign=0)
        source_label.add_css_class("heading")
        root.append(source_label)

        source_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        root.append(source_row)

        self.source_entry = Gtk.Entry(hexpand=True)
        self.source_entry.set_placeholder_text("Path to the replacement image…")
        source_row.append(self.source_entry)

        browse_btn = Gtk.Button(label="Browse…")
        browse_btn.connect("clicked", self.on_browse_source_clicked)
        source_row.append(browse_btn)

        note = Gtk.Label(
            label="Each destination image keeps its own format (.png stays .png, .jpg stays "
                  ".jpg, etc). The source image is converted to match each destination automatically.",
            xalign=0, wrap=True
        )
        note.add_css_class("dim-label")
        note.add_css_class("caption")
        root.append(note)


        run_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        root.append(run_row)

        self.run_btn = Gtk.Button(label="Run Replacement")
        self.run_btn.add_css_class("suggested-action")
        self.run_btn.add_css_class("pill")
        self.run_btn.connect("clicked", self.on_run_clicked)
        run_row.append(self.run_btn)

        self.spinner = Gtk.Spinner()
        run_row.append(self.spinner)


        log_label = Gtk.Label(label="Log", xalign=0)
        log_label.add_css_class("heading")
        root.append(log_label)

        log_scroller = Gtk.ScrolledWindow(vexpand=True)
        log_scroller.add_css_class("card")
        root.append(log_scroller)

        self.log_buffer = Gtk.TextBuffer()
        self.log_view = Gtk.TextView(buffer=self.log_buffer, editable=False, monospace=True)
        self.log_view.set_margin_top(8)
        self.log_view.set_margin_bottom(8)
        self.log_view.set_margin_start(8)
        self.log_view.set_margin_end(8)
        log_scroller.set_child(self.log_view)

        default_source = Path(__file__).resolve().parent / SOURCE_IMAGE_NAME
        if default_source.is_file():
            self.source_entry.set_text(str(default_source))



    def _setup_folder_row(self, factory, list_item):
        label = Gtk.Label(xalign=0)
        label.set_margin_top(6)
        label.set_margin_bottom(6)
        label.set_margin_start(10)
        label.set_margin_end(10)
        label.set_ellipsize(3)
        list_item.set_child(label)

    def _bind_folder_row(self, factory, list_item):
        label = list_item.get_child()
        string_obj = list_item.get_item()
        label.set_text(string_obj.get_string())



    def _add_folder_paths(self, paths):
        added = 0
        for p in paths:
            resolved = str(Path(p).expanduser().resolve())
            if Path(resolved).is_dir() and resolved not in [str(f) for f in self.target_folders]:
                self.target_folders.append(Path(resolved))
                self.folder_store.append(resolved)
                added += 1
        return added

    def on_add_folders_clicked(self, button):
        dialog = Gtk.FileDialog(title="Select one or more folders")
        dialog.select_multiple_folders(self, None, self._on_folders_selected)

    def _on_folders_selected(self, dialog, result):
        try:
            files = dialog.select_multiple_folders_finish(result)
        except GLib.Error:
            return
        paths = [f.get_path() for f in files if f.get_path()]
        added = self._add_folder_paths(paths)
        if added == 0 and paths:
            self._show_toast_or_alert("No new folders were added (already in the list?).")

    def on_paste_paths_clicked(self, button):
        paste_win = Gtk.Window(transient_for=self, modal=True, title="Paste folder paths")
        paste_win.set_default_size(480, 380)

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_top(12)
        box.set_margin_bottom(12)
        box.set_margin_start(12)
        box.set_margin_end(12)
        paste_win.set_child(box)

        info = Gtk.Label(
            label="Paste one folder path per line:",
            xalign=0, wrap=True
        )
        box.append(info)

        scroller = Gtk.ScrolledWindow(vexpand=True)
        box.append(scroller)
        text_buffer = Gtk.TextBuffer()
        text_view = Gtk.TextView(buffer=text_buffer, monospace=True)
        scroller.set_child(text_view)

        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8, halign=Gtk.Align.END)
        box.append(btn_row)

        cancel_btn = Gtk.Button(label="Cancel")
        cancel_btn.connect("clicked", lambda b: paste_win.close())
        btn_row.append(cancel_btn)

        add_btn = Gtk.Button(label="Add")
        add_btn.add_css_class("suggested-action")

        def confirm(b):
            start, end = text_buffer.get_bounds()
            raw = text_buffer.get_text(start, end, False)
            lines = [line.strip().strip('"') for line in raw.splitlines() if line.strip()]
            added = self._add_folder_paths(lines)
            paste_win.close()
            if added == 0:
                self._show_toast_or_alert("No valid folder paths were found in the pasted text.")

        add_btn.connect("clicked", confirm)
        btn_row.append(add_btn)

        paste_win.present()

    def on_remove_selected_clicked(self, button):
        bitset = self.folder_selection.get_selection()
        indices = []
        ok, it, value = Gtk.BitsetIter.init_first(bitset)
        while ok:
            indices.append(value)
            ok, value = it.next()
        for i in sorted(indices, reverse=True):
            self.folder_store.remove(i)
            del self.target_folders[i]

    def on_clear_folders_clicked(self, button):
        self.target_folders.clear()
        n = self.folder_store.get_n_items()
        if n:
            self.folder_store.splice(0, n, [])



    def on_browse_source_clicked(self, button):
        dialog = Gtk.FileDialog(title="Select the source image")
        filt = Gtk.FileFilter(name="Image files")
        for ext in sorted(SUPPORTED_EXTENSIONS):
            filt.add_pattern(f"*{ext}")
        filters = Gio.ListStore.new(Gtk.FileFilter)
        filters.append(filt)
        dialog.set_filters(filters)
        dialog.open(self, None, self._on_source_selected)

    def _on_source_selected(self, dialog, result):
        try:
            file = dialog.open_finish(result)
        except GLib.Error:
            return
        path = file.get_path()
        if path:
            self.source_entry.set_text(path)



    def _log(self, text):
        end_iter = self.log_buffer.get_end_iter()
        self.log_buffer.insert(end_iter, text)
        mark = self.log_buffer.get_insert()
        self.log_view.scroll_mark_onscreen(mark)

    def _log_threadsafe(self, text):
        GLib.idle_add(self._log, text)

    def on_run_clicked(self, button):
        if self.run_thread and self.run_thread.is_alive():

            self.run_cancelled = True
            return

        source_text = self.source_entry.get_text().strip()
        if not self.target_folders:
            self._show_toast_or_alert("Please add at least one target folder.")
            return
        if not source_text or not Path(source_text).is_file():
            self._show_toast_or_alert("Please select a valid source image.")
            return

        for folder in self.target_folders:
            if not folder.is_dir():
                self._show_toast_or_alert(f"Folder no longer exists:\n{folder}")
                return

        self.log_buffer.set_text("")
        self.run_cancelled = False
        self.spinner.start()
        self.run_btn.set_label("Cancel")

        folders = list(self.target_folders)
        source_path = Path(source_text).resolve()

        def worker():
            run_replacement(folders, source_path, self._log_threadsafe, lambda: self.run_cancelled)
            GLib.idle_add(self._on_run_finished)

        self.run_thread = threading.Thread(target=worker, daemon=True)
        self.run_thread.start()

    def _on_run_finished(self):
        self.spinner.stop()
        self.run_btn.set_label("Run Replacement")



    def _show_toast_or_alert(self, message):
        alert = Adw.AlertDialog(heading="Image Replacer", body=message)
        alert.add_response("ok", "OK")
        alert.present(self)


class ImageReplacerApp(Adw.Application):
    def __init__(self):
        super().__init__(application_id="com.example.ImageReplacer")

    def do_activate(self):
        win = self.props.active_window
        if not win:
            win = ImageReplacerWindow(self)
        win.present()


if __name__ == "__main__":
    app = ImageReplacerApp()
    app.run(sys.argv)