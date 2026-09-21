"""Browse server-side YAML files, preserving adjacent manifests and asset paths."""

from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Any

from nicegui import run, ui
from nicegui.events import GenericEventArguments


async def pick_dataset(directory: Path, on_load: Callable[[Path], Awaitable[None]]) -> None:
    def entries(folder: Path) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for path in folder.iterdir():
            is_directory = path.is_dir()
            if is_directory or path.suffix.lower() in {".yaml", ".yml"}:
                rows.append({
                    "name": path.name,
                    "kind": "Folder" if is_directory else "YAML",
                    "path": str(path),
                    "directory": is_directory,
                })
        return sorted(rows, key=lambda row: (not row["directory"], row["name"].lower()))

    async def browse(folder: Path) -> None:
        nonlocal directory
        try:
            folder = folder.expanduser().resolve(strict=True)
            rows = await run.io_bound(entries, folder)
        except (OSError, RuntimeError) as error:
            error_label.set_text(str(error))
            return
        directory = folder
        path_input.set_value(str(folder))
        table.rows = rows
        table.pagination = {"rowsPerPage": 12, "page": 1}
        error_label.set_text("")

    async def row_clicked(event: GenericEventArguments) -> None:
        row = event.args[1]
        if row["directory"]:
            await browse(Path(row["path"]))
        else:
            path_input.set_value(row["path"])
            error_label.set_text("")

    async def open_path() -> None:
        if not load_button.enabled:
            return
        if not path_input.value.strip():
            error_label.set_text("Select a YAML file or enter its path")
            return
        source = Path(path_input.value.strip()).expanduser()
        if not source.is_absolute():
            source = directory / source
        try:
            source = source.resolve(strict=True)
            if source.is_dir():
                await browse(source)
                return
            if not source.is_file() or source.suffix.lower() not in {".yaml", ".yml"}:
                raise ValueError("Select a .yaml or .yml grasp dataset")
            load_button.set_enabled(False)
            cancel_button.set_enabled(False)
            load_button.props("loading")
            # Keep the dialog open and the current scene intact on validation errors.
            await on_load(source)
        except Exception as error:
            error_label.set_text(str(error))
        else:
            dialog.close()
        finally:
            load_button.set_enabled(True)
            cancel_button.set_enabled(True)
            load_button.props(remove="loading")

    with ui.dialog().props("persistent") as dialog, ui.card().classes("dataset-dialog"):
        ui.label("Open grasp dataset").classes("text-h6")
        ui.label("Choose a YAML file on the server running Grasp Studio.").classes("muted small")
        path_input = ui.input("File or folder path", value=str(directory)).props(
            "outlined dense"
        ).classes("w-full")
        path_input.on("keydown.enter", open_path)
        ui.button(
            "Parent folder", icon="arrow_upward", on_click=lambda: browse(directory.parent)
        ).props("flat no-caps")
        table = ui.table(
            columns=[
                {"name": "kind", "label": "Type", "field": "kind", "align": "left"},
                {"name": "name", "label": "Name", "field": "name", "align": "left"},
            ],
            rows=[], row_key="path", pagination=12,
        ).classes("w-full cursor-pointer").props("dense flat")
        table.on("rowClick", row_clicked)
        error_label = ui.label("").classes("text-negative text-sm break-all")
        with ui.row().classes("w-full justify-end"):
            cancel_button = ui.button("Cancel", on_click=dialog.close).props("flat no-caps")
            load_button = ui.button("Open", icon="folder_open", on_click=open_path).props("no-caps")
    await browse(directory)
    await dialog
    dialog.delete()
