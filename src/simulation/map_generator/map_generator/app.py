"""Tkinter GUI for the standalone track map generator.

Draw a centerline with mouse clicks -> real-time Catmull-Rom spline preview
-> left/right track edges -> cone placement -> CSV export.
Also supports opening an existing cones CSV for preview.
"""
import copy
import math
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np

from map_generator import spline
from map_generator import track_generator
from map_generator import cone_generator
from map_generator import csv_io
from map_generator.geometry import polyline_length

POINT_HIT_RADIUS_PX = 10


class MapGeneratorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("F1TENTH Map Generator")
        self.geometry("1100x750")

        # --- editor state (meters) ---
        self.control_points = []
        self.mode = tk.StringVar(value="draw")
        self.closed = tk.BooleanVar(value=False)
        self.track_width = tk.DoubleVar(value=3.0)
        self.cone_spacing = tk.DoubleVar(value=1.0)
        self.spline_resolution = tk.DoubleVar(value=0.05)

        self.undo_stack = []
        self.redo_stack = []
        self.dragging_idx = None

        # view transform: pixel = pan + meter * scale (y flipped)
        self.scale = 40.0
        self.pan_x = 150.0
        self.pan_y = 400.0
        self._pan_drag_origin = None

        # imported-CSV preview state
        self.imported_pairs = None

        self._build_ui()
        self._redraw_editor()

    # ---------------------------------------------------------------- UI --
    def _build_ui(self):
        self.editor_frame = ttk.Frame(self)
        self.preview_frame = ttk.Frame(self)
        self.import_frame = ttk.Frame(self)

        self._build_editor_frame()
        self._build_preview_frame()
        self._build_import_frame()

        self.editor_frame.pack(fill=tk.BOTH, expand=True)

    def _build_editor_frame(self):
        f = self.editor_frame
        toolbar = ttk.Frame(f)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)

        ttk.Button(toolbar, text="Open CSV", command=self.on_open_csv).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Export CSV", command=self.on_export_csv).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Clear", command=self.on_clear).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Undo", command=self.on_undo).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Redo", command=self.on_redo).pack(side=tk.LEFT, padx=2)

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Label(toolbar, text="Track Width (m):").pack(side=tk.LEFT)
        ttk.Entry(toolbar, textvariable=self.track_width, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(toolbar, text="Cone Spacing (m):").pack(side=tk.LEFT)
        ttk.Entry(toolbar, textvariable=self.cone_spacing, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(toolbar, text="Spline Res (m):").pack(side=tk.LEFT)
        ttk.Entry(toolbar, textvariable=self.spline_resolution, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(toolbar, text="Closed track", variable=self.closed,
                        command=self._redraw_editor).pack(side=tk.LEFT, padx=6)

        for var, val in [(self.track_width, None), (self.cone_spacing, None), (self.spline_resolution, None)]:
            var.trace_add("write", lambda *a: self._redraw_editor())

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        for label, value in [("Draw Mode", "draw"), ("Move Point", "move"), ("Delete Point", "delete")]:
            ttk.Radiobutton(toolbar, text=label, value=value, variable=self.mode).pack(side=tk.LEFT, padx=2)

        ttk.Button(toolbar, text="NEXT ->", command=self.on_next).pack(side=tk.RIGHT, padx=4)

        self.canvas = tk.Canvas(f, background="white")
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4)
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.canvas.bind("<Button-3>", self._on_pan_start)
        self.canvas.bind("<B3-Motion>", self._on_pan_drag)
        self.canvas.bind("<Button-4>", lambda e: self._zoom(e, 1.1))
        self.canvas.bind("<Button-5>", lambda e: self._zoom(e, 1 / 1.1))
        self.canvas.bind("<MouseWheel>", lambda e: self._zoom(e, 1.1 if e.delta > 0 else 1 / 1.1))
        self.canvas.bind("<Configure>", lambda e: self._redraw_editor())

        self.status_var = tk.StringVar(value="Length: 0.0 m   Cones: 0")
        ttk.Label(f, textvariable=self.status_var, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)

    def _build_preview_frame(self):
        f = self.preview_frame
        toolbar = ttk.Frame(f)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)
        ttk.Button(toolbar, text="<- Back", command=self.on_back_from_preview).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Export CSV", command=self.on_export_csv).pack(side=tk.LEFT, padx=2)

        self.preview_canvas = tk.Canvas(f, background="white")
        self.preview_canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4)
        self.preview_canvas.bind("<Configure>", lambda e: self._redraw_preview())

        self.preview_status_var = tk.StringVar(value="")
        ttk.Label(f, textvariable=self.preview_status_var, anchor=tk.W, justify=tk.LEFT).pack(
            side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)

    def _build_import_frame(self):
        f = self.import_frame
        toolbar = ttk.Frame(f)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)
        ttk.Button(toolbar, text="<- Back to editor", command=self.on_back_from_import).pack(side=tk.LEFT, padx=2)
        ttk.Label(toolbar, text="Imported CSV preview (read-only)").pack(side=tk.LEFT, padx=8)

        self.import_canvas = tk.Canvas(f, background="white")
        self.import_canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4)
        self.import_canvas.bind("<Configure>", lambda e: self._redraw_import())

        self.import_status_var = tk.StringVar(value="")
        ttk.Label(f, textvariable=self.import_status_var, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)

    # ------------------------------------------------------- coordinates --
    def m2p(self, x, y):
        return self.pan_x + x * self.scale, self.pan_y - y * self.scale

    def p2m(self, px, py):
        return (px - self.pan_x) / self.scale, (self.pan_y - py) / self.scale

    def _zoom(self, event, factor):
        mx, my = self.p2m(event.x, event.y)
        self.scale *= factor
        self.scale = max(2.0, min(self.scale, 400.0))
        self.pan_x = event.x - mx * self.scale
        self.pan_y = event.y + my * self.scale
        self._redraw_editor()

    def _on_pan_start(self, event):
        self._pan_drag_origin = (event.x, event.y, self.pan_x, self.pan_y)

    def _on_pan_drag(self, event):
        if self._pan_drag_origin is None:
            return
        ox, oy, opx, opy = self._pan_drag_origin
        self.pan_x = opx + (event.x - ox)
        self.pan_y = opy + (event.y - oy)
        self._redraw_editor()

    # -------------------------------------------------------- undo/redo --
    def _snapshot(self):
        self.undo_stack.append(copy.deepcopy(self.control_points))
        self.redo_stack.clear()

    def on_undo(self):
        if not self.undo_stack:
            return
        self.redo_stack.append(copy.deepcopy(self.control_points))
        self.control_points = self.undo_stack.pop()
        self._redraw_editor()

    def on_redo(self):
        if not self.redo_stack:
            return
        self.undo_stack.append(copy.deepcopy(self.control_points))
        self.control_points = self.redo_stack.pop()
        self._redraw_editor()

    def on_clear(self):
        if not self.control_points:
            return
        self._snapshot()
        self.control_points = []
        self._redraw_editor()

    # ------------------------------------------------------ mouse events --
    def _nearest_point_idx(self, px, py):
        best_idx, best_d = None, POINT_HIT_RADIUS_PX
        for i, (x, y) in enumerate(self.control_points):
            cx, cy = self.m2p(x, y)
            d = ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5
            if d < best_d:
                best_d = d
                best_idx = i
        return best_idx

    def _on_canvas_click(self, event):
        mode = self.mode.get()
        if mode == "draw":
            self._snapshot()
            mx, my = self.p2m(event.x, event.y)
            self.control_points.append((mx, my))
            self._redraw_editor()
        elif mode == "move":
            idx = self._nearest_point_idx(event.x, event.y)
            if idx is not None:
                self._snapshot()
                self.dragging_idx = idx
        elif mode == "delete":
            idx = self._nearest_point_idx(event.x, event.y)
            if idx is not None:
                self._snapshot()
                del self.control_points[idx]
                self._redraw_editor()

    def _on_canvas_drag(self, event):
        if self.mode.get() == "move" and self.dragging_idx is not None:
            mx, my = self.p2m(event.x, event.y)
            self.control_points[self.dragging_idx] = (mx, my)
            self._redraw_editor()

    def _on_canvas_release(self, event):
        self.dragging_idx = None

    # -------------------------------------------------------- computation --
    def _compute_track(self):
        width = max(0.1, self.track_width.get())
        resolution = max(0.01, self.spline_resolution.get())
        spacing = max(0.05, self.cone_spacing.get())
        closed = self.closed.get()

        centerline = spline.generate_centerline(self.control_points, resolution, closed)
        left, right = track_generator.compute_edges(centerline, width, closed)
        pairs = cone_generator.generate_cone_pairs(centerline, left, right, spacing, closed)
        return centerline, left, right, pairs

    # ------------------------------------------------------------ drawing --
    def _draw_grid(self, c, w, h):
        """Meter grid, spacing adapted to zoom (~25-100px between lines),
        with the origin axes drawn heavier and grid lines labeled in meters.
        """
        target_px = 50.0
        raw_spacing_m = target_px / self.scale
        magnitude = 10 ** math.floor(math.log10(raw_spacing_m))
        for step in (1, 2, 5, 10):
            spacing_m = step * magnitude
            if spacing_m >= raw_spacing_m:
                break

        x0, _ = self.p2m(0, h)
        x1, _ = self.p2m(w, 0)
        _, y0 = self.p2m(0, h)
        _, y1 = self.p2m(0, 0)

        start_x = math.floor(x0 / spacing_m) * spacing_m
        gx = start_x
        while gx <= x1:
            px, _ = self.m2p(gx, 0)
            is_axis = abs(gx) < spacing_m * 1e-6
            c.create_line(px, 0, px, h, fill="#4a5568" if is_axis else "#e0e0e0",
                          width=2 if is_axis else 1)
            if not is_axis:
                c.create_text(px + 3, h - 10, text=f"{gx:g}", anchor="w",
                              fill="#a0aec0", font=("TkDefaultFont", 7))
            gx += spacing_m

        start_y = math.floor(y0 / spacing_m) * spacing_m
        gy = start_y
        while gy <= y1:
            _, py = self.m2p(0, gy)
            is_axis = abs(gy) < spacing_m * 1e-6
            c.create_line(0, py, w, py, fill="#4a5568" if is_axis else "#e0e0e0",
                          width=2 if is_axis else 1)
            if not is_axis:
                c.create_text(3, py - 8, text=f"{gy:g}", anchor="w",
                              fill="#a0aec0", font=("TkDefaultFont", 7))
            gy += spacing_m

    def _redraw_editor(self):
        c = self.canvas
        c.delete("all")

        w = c.winfo_width() or 1
        h = c.winfo_height() or 1
        self._draw_grid(c, w, h)

        centerline = np.zeros((0, 2))
        pairs = []
        if len(self.control_points) >= 2:
            centerline, left, right, pairs = self._compute_track()

            if len(left) >= 2:
                c.create_line(*[v for p in left for v in self.m2p(*p)], fill="#2b6cb0", width=2)
            if len(right) >= 2:
                c.create_line(*[v for p in right for v in self.m2p(*p)], fill="#d69e2e", width=2)
            if len(centerline) >= 2:
                c.create_line(*[v for p in centerline for v in self.m2p(*p)], fill="#a0aec0", dash=(4, 3))

        # control points + polyline between them (for reference while drawing)
        if len(self.control_points) >= 2:
            c.create_line(*[v for p in self.control_points for v in self.m2p(*p)], fill="#cbd5e0", dash=(2, 2))
        for i, (x, y) in enumerate(self.control_points):
            px, py = self.m2p(x, y)
            c.create_rectangle(px - 4, py - 4, px + 4, py + 4, fill="#2d3748", outline="")

        length = polyline_length(centerline) if len(centerline) else 0.0
        self.status_var.set(f"Length: {length:.1f} m   Cones: {len(pairs) * 2}")

    def _redraw_preview(self):
        c = self.preview_canvas
        c.delete("all")
        if len(self.control_points) < 2:
            return
        centerline, left, right, pairs = self._compute_track()

        if len(left) >= 2:
            c.create_line(*[v for p in left for v in self.m2p(*p)], fill="#2b6cb0", width=2)
        if len(right) >= 2:
            c.create_line(*[v for p in right for v in self.m2p(*p)], fill="#d69e2e", width=2)

        for pair in pairs:
            bx, by = self.m2p(*pair["blue"])
            yx, yy = self.m2p(*pair["yellow"])
            c.create_oval(bx - 5, by - 5, bx + 5, by + 5, fill="#2b6cb0", outline="")
            c.create_oval(yx - 5, yy - 5, yx + 5, yy + 5, fill="#d69e2e", outline="")

        length = polyline_length(centerline)
        avg_spacing = length / max(1, len(pairs) - (0 if self.closed.get() else 1)) if len(pairs) > 1 else 0.0
        self.preview_status_var.set(
            f"Track length: {length:.1f} m\n"
            f"Total cones: {len(pairs) * 2}\n"
            f"Average spacing: {avg_spacing:.2f} m\n"
            f"Track width: {self.track_width.get():.2f} m"
        )

    def _redraw_import(self):
        c = self.import_canvas
        c.delete("all")
        if not self.imported_pairs:
            return

        blue_pts = [p["blue"] for p in self.imported_pairs if p["blue"] is not None]
        yellow_pts = [p["yellow"] for p in self.imported_pairs if p["yellow"] is not None]
        centers = csv_io.reconstruct_centerline(self.imported_pairs)

        if len(blue_pts) >= 2:
            c.create_line(*[v for p in blue_pts for v in self.m2p(*p)], fill="#2b6cb0", width=2)
        if len(yellow_pts) >= 2:
            c.create_line(*[v for p in yellow_pts for v in self.m2p(*p)], fill="#d69e2e", width=2)
        if len(centers) >= 2:
            c.create_line(*[v for p in centers for v in self.m2p(*p)], fill="#a0aec0", dash=(4, 3))

        for p in blue_pts:
            px, py = self.m2p(*p)
            c.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#2b6cb0", outline="")
        for p in yellow_pts:
            px, py = self.m2p(*p)
            c.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#d69e2e", outline="")

        length = polyline_length(np.array(centers)) if len(centers) >= 2 else 0.0
        self.import_status_var.set(
            f"Imported cones: {len(blue_pts) + len(yellow_pts)}   "
            f"Pairs: {len(self.imported_pairs)}   Est. length: {length:.1f} m")

    # ------------------------------------------------------------ actions --
    def on_next(self):
        if len(self.control_points) < 2:
            messagebox.showinfo("Map Generator", "Draw at least 2 points first.")
            return
        self.editor_frame.pack_forget()
        self.preview_frame.pack(fill=tk.BOTH, expand=True)
        self._redraw_preview()

    def on_back_from_preview(self):
        self.preview_frame.pack_forget()
        self.editor_frame.pack(fill=tk.BOTH, expand=True)
        self._redraw_editor()

    def on_back_from_import(self):
        self.import_frame.pack_forget()
        self.editor_frame.pack(fill=tk.BOTH, expand=True)
        self._redraw_editor()

    def on_export_csv(self):
        if len(self.control_points) < 2:
            messagebox.showinfo("Map Generator", "Draw at least 2 points first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                             filetypes=[("CSV files", "*.csv")])
        if not path:
            return
        _, _, _, pairs = self._compute_track()
        csv_io.export_csv(path, pairs)
        messagebox.showinfo("Map Generator", f"Exported {len(pairs) * 2} cones to:\n{path}")

    def on_open_csv(self):
        path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if not path:
            return
        try:
            self.imported_pairs = csv_io.import_csv(path)
        except Exception as exc:
            messagebox.showerror("Map Generator", f"Failed to load CSV:\n{exc}")
            return
        self.editor_frame.pack_forget()
        self.preview_frame.pack_forget()
        self.import_frame.pack(fill=tk.BOTH, expand=True)
        self._redraw_import()


def main():
    app = MapGeneratorApp()
    app.mainloop()


if __name__ == "__main__":
    main()
