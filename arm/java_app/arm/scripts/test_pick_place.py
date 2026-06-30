import socket
import sys

def main():
    # UDP setup to communicate with the Java app
    udp_ip = "127.0.0.1"
    udp_port = 5005

    if len(sys.argv) < 4:
        print("Cách dùng: python test_pick_place.py <X> <Y> <Z>")
        print("Ví dụ (gắp vật ở X=120, Y=0, Z=20):")
        print("  python test_pick_place.py 120 0 20")
        
        # Default coordinates for a quick run if no args provided
        x = 120.0
        y = 0.0
        z = 20.0
        print(f"\nKhông có tham số, sử dụng tọa độ mặc định: X={x}, Y={y}, Z={z}")
    else:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
        except ValueError:
            print("Lỗi: X, Y, Z phải là các số thực.")
            sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    msg = f"PICK: {x}, {y}, {z}"
    
    try:
        sock.sendto(msg.encode('utf-8'), (udp_ip, udp_port))
        print(f" Đã gửi lệnh gắp qua UDP tới cổng {udp_port} thành công!")
        print(f" -> Tọa độ mục tiêu gửi đi: X={x}, Y={y}, Z={z}")
        print("Hãy kiểm tra mô phỏng 3D hoặc cánh tay thật xem đã chạy chu trình gắp chưa.")
    except Exception as e:
        print(f"Lỗi khi gửi dữ liệu UDP: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
