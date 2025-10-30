# Module status message for health monitoring

string module_name    # Module name (head, ears_neck, body, base)
uint8 status_code     # 0=OK, 1=WARNING, 2=ERROR, 3=OFFLINE
string status_message # Human-readable status description
uint32 uptime_ms      # Module uptime in milliseconds
float32 cpu_usage     # CPU usage percentage (0-100)
float32 memory_usage  # Memory usage percentage (0-100)
