"""Staff records: lookup and CRUD."""

from .db import db_connect

__all__ = [
    "lookup_staff_by_device", "fetch_all_staff",
    "add_staff", "update_staff", "delete_staff",
]


def lookup_staff_by_device(device_id: str):
    conn = db_connect()
    if not conn:
        return None
    try:
        row = conn.execute("""
            SELECT s.name, s.role, s.email, s.phone FROM devices d
            JOIN staff s ON d.assigned_staff_id = s.id WHERE d.device_id = ?
        """, (device_id,)).fetchone()
        return dict(row) if row else None
    finally:
        conn.close()


def fetch_all_staff():
    conn = db_connect()
    if not conn:
        return []
    try:
        return [dict(r) for r in conn.execute("SELECT * FROM staff ORDER BY name ASC").fetchall()]
    finally:
        conn.close()


def add_staff(name, role, email, phone,
              email_alerts_mass=False, email_alerts_personal=False,
              sms_alerts_mass=False, sms_alerts_personal=False):
    conn = db_connect()
    try:
        conn.execute(
            "INSERT INTO staff (name, role, email, phone, email_alerts_mass, "
            "email_alerts_personal, sms_alerts_mass, sms_alerts_personal) "
            "VALUES (?,?,?,?,?,?,?,?)",
            (name, role, email, phone,
             int(email_alerts_mass), int(email_alerts_personal),
             int(sms_alerts_mass), int(sms_alerts_personal)))
        conn.commit()
    finally:
        conn.close()


def update_staff(staff_id, name, role, email, phone,
                 email_alerts_mass, email_alerts_personal,
                 sms_alerts_mass, sms_alerts_personal,
                 admin_email_low_battery=None,
                 admin_sms_low_battery=None,
                 admin_email_ups=None,
                 admin_sms_ups=None,
                 admin_email_heartbeat_fail=None,
                 admin_sms_heartbeat_fail=None):
    """Update a staff record.

    Admin alert fields default to None, meaning "keep the current value in the
    DB", so forms that don't manage those flags don't accidentally reset them.
    """
    conn = db_connect()
    try:
        if any(x is None for x in [admin_email_low_battery, admin_sms_low_battery,
                                   admin_email_ups, admin_sms_ups,
                                   admin_email_heartbeat_fail, admin_sms_heartbeat_fail]):
            row = conn.execute(
                "SELECT admin_email_low_battery, admin_sms_low_battery, "
                "admin_email_ups, admin_sms_ups, "
                "admin_email_heartbeat_fail, admin_sms_heartbeat_fail FROM staff WHERE id=?",
                (staff_id,)
            ).fetchone()
            if row:
                if admin_email_low_battery is None: admin_email_low_battery = row[0] or 0
                if admin_sms_low_battery   is None: admin_sms_low_battery   = row[1] or 0
                if admin_email_ups         is None: admin_email_ups         = row[2] or 0
                if admin_sms_ups           is None: admin_sms_ups           = row[3] or 0
                if admin_email_heartbeat_fail is None: admin_email_heartbeat_fail = row[4] or 0
                if admin_sms_heartbeat_fail   is None: admin_sms_heartbeat_fail   = row[5] or 0
            else:
                admin_email_low_battery = admin_email_low_battery or 0
                admin_sms_low_battery   = admin_sms_low_battery   or 0
                admin_email_ups         = admin_email_ups         or 0
                admin_sms_ups           = admin_sms_ups           or 0
                admin_email_heartbeat_fail = admin_email_heartbeat_fail or 0
                admin_sms_heartbeat_fail   = admin_sms_heartbeat_fail   or 0

        conn.execute(
            "UPDATE staff SET name=?, role=?, email=?, phone=?, "
            "email_alerts_mass=?, email_alerts_personal=?, "
            "sms_alerts_mass=?, sms_alerts_personal=?, "
            "admin_email_low_battery=?, admin_sms_low_battery=?, "
            "admin_email_ups=?, admin_sms_ups=?, "
            "admin_email_heartbeat_fail=?, admin_sms_heartbeat_fail=? "
            "WHERE id=?",
            (name, role, email, phone,
             int(email_alerts_mass), int(email_alerts_personal),
             int(sms_alerts_mass), int(sms_alerts_personal),
             int(admin_email_low_battery), int(admin_sms_low_battery),
             int(admin_email_ups), int(admin_sms_ups),
             int(admin_email_heartbeat_fail), int(admin_sms_heartbeat_fail),
             staff_id)
        )
        conn.commit()
    finally:
        conn.close()


def delete_staff(staff_id):
    conn = db_connect()
    try:
        conn.execute("UPDATE devices SET assigned_staff_id=NULL WHERE assigned_staff_id=?", (staff_id,))
        conn.execute("DELETE FROM staff WHERE id=?", (staff_id,))
        conn.commit()
    finally:
        conn.close()
