"""Shared CSS and small HTML snippet builders for the dashboard pages."""

from .config import DRILL_SOURCE

__all__ = [
    "ERS_CSS", "pill", "status_pill", "type_badge", "dot_cls", "row_cls", "ers_header",
]

ERS_CSS = """
<style>
@import url('https://fonts.googleapis.com/css2?family=Bebas+Neue&family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans:wght@300;400;600&display=swap');

:root {
    --bg:      #ffffff;
    --surface: #ffffff;
    --border:  #1e2530;
    --accent:  #4065a1;
    --warn:    #f4a261;
    --ok:      #2ec4b6;
    --drill:   #f7b731;
    --text:    #111111;
    --muted:   #586069;
}
html, body, [data-testid="stAppViewContainer"] {
    background: var(--bg) !important;
    color: var(--text) !important;
    font-family: 'IBM Plex Sans', sans-serif !important;
}
[data-testid="stSidebar"] {
    background: var(--surface) !important;
    border-right: 1px solid var(--border) !important;
}
.ers-header {
    display: flex; align-items: baseline; gap: 18px;
    padding: 10px 0 24px 0;
    border-bottom: 1px solid var(--border);
    margin-bottom: 28px;
}
.ers-logo {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 3rem; letter-spacing: 6px;
    color: var(--accent); line-height: 1;
}
.ers-sub {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.7rem; color: var(--muted);
    letter-spacing: 3px; text-transform: uppercase; padding-bottom: 4px;
}
.kpi-row { display: flex; gap: 16px; margin-bottom: 28px; flex-wrap: wrap; }
.kpi-card {
    flex: 1; min-width: 120px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 6px; padding: 16px 20px;
    position: relative; overflow: hidden;
}
.kpi-card::before {
    content: ''; position: absolute;
    left: 0; top: 0; bottom: 0; width: 3px;
}
.kpi-card.red::before   { background: var(--accent); }
.kpi-card.amber::before { background: var(--warn); }
.kpi-card.drill::before { background: var(--drill); }
.kpi-card.gray::before  { background: #30363d; }
.kpi-num {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 2.4rem; line-height: 1; color: #111;
}
.kpi-label {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.65rem; color: var(--muted);
    letter-spacing: 2px; text-transform: uppercase; margin-top: 4px;
}
.status-grid { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 24px; }
.status-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 6px; padding: 14px 18px; min-width: 150px; flex: 1;
}
.status-card-label {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.6rem; color: var(--muted);
    letter-spacing: 2px; text-transform: uppercase; margin-bottom: 8px;
}
.status-card-detail {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.65rem; color: var(--muted); margin-top: 6px;
}
.tbl-header {
    display: flex; gap: 10px; padding: 8px 14px;
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.6rem; letter-spacing: 2px;
    color: var(--muted); text-transform: uppercase;
    border-bottom: 1px solid var(--border); margin-bottom: 4px;
}
.inc-row {
    display: flex; align-items: center; gap: 10px;
    padding: 11px 14px;
    border-bottom: 1px solid var(--border);
    border-radius: 4px; transition: background 0.15s;
    font-size: 0.82rem;
}
.inc-row:hover { background: #f5f7fa; }
.inc-row.is-drill { border-left: 3px solid var(--drill); }
.inc-row.is-mass  { border-left: 3px solid var(--accent); }
.badge {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.62rem; font-weight: 600;
    letter-spacing: 1px; padding: 3px 8px;
    border-radius: 3px; text-transform: uppercase; white-space: nowrap;
}
.badge-mass     { background: rgba(230,57,70,0.18);  color: #e63946; border: 1px solid rgba(230,57,70,0.4); }
.badge-personal { background: rgba(244,162,97,0.15); color: #f4a261; border: 1px solid rgba(244,162,97,0.4); }
.badge-drill    { background: rgba(247,183,49,0.15); color: #f7b731; border: 1px solid rgba(247,183,49,0.4); }
.pill {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.58rem; padding: 2px 6px;
    border-radius: 3px; white-space: nowrap; display: inline-block;
}
.pill-ok      { background: rgba(46,196,182,0.12);  color: #2ec4b6; }
.pill-warn    { background: rgba(247,183,49,0.12);  color: #f7b731; }
.pill-fail    { background: rgba(230,57,70,0.12);   color: #e63946; }
.pill-skip    { background: rgba(88,96,105,0.2);    color: #586069; }
.pill-unknown { background: rgba(88,96,105,0.15);   color: #586069; }
.status-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; }
.dot-mass     { background: var(--accent); box-shadow: 0 0 4px var(--accent); }
.dot-personal { background: var(--warn); }
.dot-drill    { background: var(--drill); box-shadow: 0 0 4px var(--drill); }
.mono { font-family: 'IBM Plex Mono', monospace; font-size: 0.7rem; color: var(--muted); }
.section-title {
    font-family: 'IBM Plex Mono', monospace; font-size: 0.65rem;
    letter-spacing: 3px; color: var(--muted); text-transform: uppercase;
    margin-bottom: 12px; padding-bottom: 8px;
    border-bottom: 1px solid var(--border);
}
.drill-alert {
    background: rgba(247,183,49,0.08);
    border: 1px solid rgba(247,183,49,0.45);
    border-left: 4px solid var(--drill);
    border-radius: 6px; padding: 16px 20px; margin-bottom: 20px;
    font-family: 'IBM Plex Mono', monospace; font-size: 0.8rem;
    color: var(--drill); letter-spacing: 1px;
}
.drill-alert strong { font-size: 1rem; letter-spacing: 3px; }
.nav-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 8px; padding: 28px 24px;
    text-decoration: none; display: block;
    transition: border-color 0.15s, box-shadow 0.15s;
    cursor: pointer;
}
.nav-card:hover { border-color: var(--accent); box-shadow: 0 0 0 1px var(--accent); }
.nav-card-title {
    font-family: 'Bebas Neue', sans-serif;
    font-size: 1.4rem; letter-spacing: 3px; color: var(--accent);
}
.nav-card-desc  {
    font-family: 'IBM Plex Mono', monospace;
    font-size: 0.65rem; color: var(--muted); margin-top: 6px;
    letter-spacing: 1px; line-height: 1.6;
}
.stButton > button {
    font-family: 'IBM Plex Mono', monospace !important;
    font-size: 0.75rem !important;
    letter-spacing: 2px !important;
    text-transform: uppercase !important;
    border-radius: 4px !important;
}
</style>
"""


def pill(status, label):
    """Render a small status pill for an incident alert channel result."""
    cls = {
        "ok": "pill-ok", "failed": "pill-fail", "queue_full": "pill-fail",
        "skipped": "pill-skip", "queued": "pill-ok",
    }.get(status, "pill-unknown")
    return f'<span class="pill {cls}">{label}</span>'


def status_pill(state, text):
    """Render a coloured status pill for a system-status indicator."""
    cls = {"ok": "pill-ok", "warn": "pill-warn", "error": "pill-fail"}.get(state, "pill-unknown")
    icon = {"ok": "●", "warn": "▲", "error": "✕"}.get(state, "•")
    return f'<span class="pill {cls}">{icon} {text}</span>'


def type_badge(em_type, src):
    """Render a DRILL / MASS / PERSONAL badge for an incident."""
    if src == DRILL_SOURCE:
        return '<span class="badge badge-drill">DRILL</span>'
    if em_type == "mass":
        return '<span class="badge badge-mass">MASS</span>'
    return '<span class="badge badge-personal">PERSONAL</span>'


def dot_cls(em_type, src):
    """Return the CSS dot class for an incident's type."""
    if src == DRILL_SOURCE:
        return "dot-drill"
    return "dot-mass" if em_type == "mass" else "dot-personal"


def row_cls(em_type, src):
    """Return the CSS row class for an incident's type."""
    if src == DRILL_SOURCE:
        return "is-drill"
    return "is-mass" if em_type == "mass" else ""


def ers_header(subtitle=""):
    """Render the page header block with an optional subtitle."""
    return f"""
    <div class="ers-header">
      <div class="ers-logo">ERS</div>
      {"<div class='ers-sub'>" + subtitle + "</div>" if subtitle else ""}
    </div>
    """
