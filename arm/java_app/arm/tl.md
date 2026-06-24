# 100 câu hỏi mở code giao diện 3D Java Swing

> Tất cả câu hỏi trong file này dùng chung một định dạng để trả lời khi cô yêu cầu mở code trực tiếp: **Vị trí mở code -> Nói khi mở code -> Dòng quan trọng -> Nếu cô yêu cầu sửa**.
>
> Trọng tâm: giao diện 3D bằng Java Swing, `JPanel`, `paintComponent`, `Graphics2D`, layout, listener, timer, repaint, và các chỗ sửa màu/kích thước trên giao diện.

---

## A. Class, package và cấu trúc giao diện 3D

### Câu 1. Class nào phụ trách vẽ giao diện 3D?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, dòng 9-10.
- **Nói khi mở code:** Đây là class phụ trách vẽ mô phỏng 3D. Class này kế thừa `JPanel` và bắt các sự kiện chuột.
- **Dòng quan trọng:** Dòng 9-10: `public class ArmPanel extends JPanel implements MouseListener, MouseMotionListener, MouseWheelListener`.
- **Nếu cô yêu cầu sửa:** Nếu muốn sửa vùng vẽ robot 3D thì mở `ArmPanel.java` trước.

### Câu 2. Class nào tạo cửa sổ chính?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, dòng 14.
- **Nói khi mở code:** `MainFrame` kế thừa `JFrame`, là cửa sổ chính của chương trình.
- **Dòng quan trọng:** Dòng 14: `public final class MainFrame extends JFrame implements ActionListener, ChangeListener`.
- **Nếu cô yêu cầu sửa:** Nếu muốn sửa bố cục cửa sổ, nút, slider, tab thì sửa trong `MainFrame.java`.

### Câu 3. Class chạy đầu tiên nằm ở đâu?
- **Vị trí mở code:** package `app`, class `App`, file `src/app/App.java`, hàm `main(...)`, dòng 18-31.
- **Nói khi mở code:** Chương trình bắt đầu từ `App.main()`, sau đó mở `MainFrame`.
- **Dòng quan trọng:** Dòng 29-31 gọi `SwingUtilities.invokeLater(...)`.
- **Nếu cô yêu cầu sửa:** Nếu muốn mở form khác đầu tiên thì sửa đối tượng tạo trong lambda này.

### Câu 4. Vì sao GUI được mở bằng `SwingUtilities.invokeLater()`?
- **Vị trí mở code:** package `app`, class `App`, file `src/app/App.java`, dòng 29-31.
- **Nói khi mở code:** Swing nên tạo và cập nhật giao diện trên Event Dispatch Thread để giao diện ổn định.
- **Dòng quan trọng:** Dòng 29-31.
- **Nếu cô yêu cầu sửa:** Không nên bỏ `invokeLater`; chỉ đổi nội dung bên trong nếu muốn mở form khác.

### Câu 5. Panel 3D được tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, constructor `MainFrame()`, dòng 94-99.
- **Nói khi mở code:** `MainFrame` tạo `armPanel = new ArmPanel(this)` rồi add vào cửa sổ.
- **Dòng quan trọng:** Dòng 95 tạo `ArmPanel`; dòng 98 add vào `BorderLayout.CENTER`.
- **Nếu cô yêu cầu sửa:** Muốn đổi vị trí vùng vẽ 3D thì sửa dòng 98.

### Câu 6. Vì sao đặt vùng vẽ 3D ở `BorderLayout.CENTER`?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, dòng 97-99.
- **Nói khi mở code:** `CENTER` là vùng lớn nhất của `BorderLayout`, phù hợp để hiển thị mô phỏng 3D.
- **Dòng quan trọng:** Dòng 98: `add(armPanel, BorderLayout.CENTER);`.
- **Nếu cô yêu cầu sửa:** Có thể đổi thành `WEST`, `EAST`, nhưng không nên vì vùng vẽ sẽ hẹp hơn.

### Câu 7. Panel điều khiển bên phải được tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildControlPanel()`, dòng 117-122.
- **Nói khi mở code:** Control panel nằm bên phải cửa sổ và chứa slider, ô nhập, tab.
- **Dòng quan trọng:** Dòng 118 add vào `BorderLayout.EAST`; dòng 120 đặt rộng `340`.
- **Nếu cô yêu cầu sửa:** Muốn panel rộng hơn thì đổi `340` thành `420`; muốn hẹp hơn thì đổi thành `300`.

### Câu 8. Thanh trên cùng của giao diện được tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildTopPanel()`, dòng 383-442.
- **Nói khi mở code:** Top panel chứa nút kẹp, thanh tốc độ và khu vực COM.
- **Dòng quan trọng:** Dòng 384 add `topPanel` vào `BorderLayout.NORTH`.
- **Nếu cô yêu cầu sửa:** Muốn thêm nút nhanh thì thêm component trong `buildTopPanel()`.

### Câu 9. Menu của chương trình được tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildMenuBar()`, dòng 444-543.
- **Nói khi mở code:** Menu gồm điều khiển, hiển thị và chế độ.
- **Dòng quan trọng:** Dòng 542: `setJMenuBar(menuBar);`.
- **Nếu cô yêu cầu sửa:** Muốn thêm menu hoặc menu item thì sửa trong `buildMenuBar()`.

### Câu 10. Kích thước cửa sổ chính sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, constructor `MainFrame()`, dòng 104.
- **Nói khi mở code:** Kích thước ban đầu của cửa sổ được đặt bằng `setSize`.
- **Dòng quan trọng:** Dòng 104: `setSize(1100, 750);`.
- **Nếu cô yêu cầu sửa:** Đổi thành `setSize(1280, 800);` nếu muốn cửa sổ lớn hơn.

---

## B. Vẽ 3D bằng `paintComponent` và `Graphics2D`

### Câu 11. Hàm vẽ chính của panel 3D là hàm nào?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(Graphics g)`, dòng 135-243.
- **Nói khi mở code:** Toàn bộ hình 3D được vẽ trong `paintComponent`.
- **Dòng quan trọng:** Dòng 135 bắt đầu hàm; dòng 136 gọi `super.paintComponent(g)`.
- **Nếu cô yêu cầu sửa:** Muốn đổi thứ tự vẽ thì sửa trong hàm `paintComponent`.

### Câu 12. Vì sao phải gọi `super.paintComponent(g)`?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, dòng 135-136.
- **Nói khi mở code:** Gọi `super` để Swing xóa nền và chuẩn bị lại vùng vẽ.
- **Dòng quan trọng:** Dòng 136: `super.paintComponent(g);`.
- **Nếu cô yêu cầu sửa:** Không nên bỏ dòng này vì có thể gây bóng hình cũ.

### Câu 13. Vì sao ép `Graphics` sang `Graphics2D`?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, dòng 137-138.
- **Nói khi mở code:** `Graphics2D` hỗ trợ vẽ đẹp hơn: chống răng cưa, nét dày, màu trong suốt.
- **Dòng quan trọng:** Dòng 137 tạo `Graphics2D g2`; dòng 138 bật antialiasing.
- **Nếu cô yêu cầu sửa:** Muốn đổi chất lượng vẽ thì sửa các `RenderingHints` ở dòng 138.

### Câu 14. Chỗ nào bật chống răng cưa?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, dòng 138.
- **Nói khi mở code:** Antialiasing giúp đường thẳng và hình tròn mịn hơn.
- **Dòng quan trọng:** Dòng 138: `g2.setRenderingHint(...)`.
- **Nếu cô yêu cầu sửa:** Có thể tắt bằng `RenderingHints.VALUE_ANTIALIAS_OFF`.

### Câu 15. Tâm màn hình để vẽ robot tính ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 139.
- **Nói khi mở code:** `cx`, `cy` là tâm chiếu để đặt robot lên panel.
- **Dòng quan trọng:** Dòng 139: `int cx = getWidth() / 2, cy = getHeight() * 2 / 3;`.
- **Nếu cô yêu cầu sửa:** Muốn robot nằm giữa dọc hơn thì đổi `cy` thành `getHeight() / 2`.

### Câu 16. Hàm chiếu điểm 3D sang 2D nằm ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `project(...)`, dòng 492-500.
- **Nói khi mở code:** Hàm này nhận điểm 3D và trả về tọa độ pixel trên màn hình.
- **Dòng quan trọng:** Dòng 493-497 tính camera; dòng 498 tính phối cảnh; dòng 499 trả về điểm 2D.
- **Nếu cô yêu cầu sửa:** Muốn đổi phối cảnh thì sửa công thức `f` ở dòng 498.

### Câu 17. Vì sao đây là 3D mà vẫn vẽ bằng Swing 2D?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `project(...)`, dòng 492-500; hàm `paintComponent(...)`, dòng 153-160.
- **Nói khi mở code:** Dữ liệu robot là tọa độ 3D, sau đó được chiếu thành điểm 2D để vẽ bằng Swing.
- **Dòng quan trọng:** Dòng 153 tính điểm 3D; dòng 157 gọi `project(...)`.
- **Nếu cô yêu cầu sửa:** Nếu muốn dùng engine 3D thật thì phải thay cả cơ chế vẽ trong `ArmPanel`.

### Câu 18. Các điểm khớp 3D được tính ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `computeAllJoints3D()`, dòng 502-526.
- **Nói khi mở code:** Hàm này tính tọa độ các khớp để vẽ robot.
- **Dòng quan trọng:** Dòng 507-512 khai báo tham số; dòng 515-518 tính điểm từng khớp.
- **Nếu cô yêu cầu sửa:** Muốn thêm/bớt khớp thì phải sửa hàm này và các mảng liên quan.

### Câu 19. Ma trận đầu kẹp hiện tại được tính ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `computeEndEffectorMatrix()`, dòng 476-489.
- **Nói khi mở code:** Hàm này trả về ma trận của đầu công tác để vẽ kẹp đúng hướng.
- **Dòng quan trọng:** Dòng 476-489.
- **Nếu cô yêu cầu sửa:** Nếu đổi quy ước tool thì kiểm tra hàm này và `getToolMatrix()` trong `Kinematics`.

### Câu 20. Vì sao phải sắp xếp gần xa khi vẽ?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 227-233.
- **Nói khi mở code:** Swing vẽ theo thứ tự lệnh, nên cần sort để vật xa/gần hiển thị hợp lý.
- **Dòng quan trọng:** Dòng 228 sort `drawables`; dòng 231-232 vẽ từng đối tượng.
- **Nếu cô yêu cầu sửa:** Nếu bị che khuất sai thì sửa comparator ở dòng 228 hoặc hàm `getDepth()`.

---

## C. Màu sắc, kích thước và thành phần 3D

### Câu 21. Nền vùng vẽ 3D đổi màu ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, constructor `ArmPanel(...)`, dòng 22-24.
- **Nói khi mở code:** Nền panel được đặt khi tạo `ArmPanel`.
- **Dòng quan trọng:** Dòng 24: `setBackground(Color.WHITE);`.
- **Nếu cô yêu cầu sửa:** Đổi thành `setBackground(new Color(220, 240, 255));` cho xanh nhạt.

### Câu 22. Màu mặt sàn đổi ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `drawGrid(...)`, dòng 422-447.
- **Nói khi mở code:** Mặt sàn được vẽ bằng polygon trong hàm `drawGrid`.
- **Dòng quan trọng:** Dòng 435: `g2.setColor(new Color(220, 220, 225));`.
- **Nếu cô yêu cầu sửa:** Đổi màu ở dòng 435.

### Câu 23. Màu đường lưới đổi ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `drawGrid(...)`, dòng 422-447.
- **Nói khi mở code:** Sau khi tô sàn, code đặt màu để vẽ các đường lưới.
- **Dòng quan trọng:** Dòng 438: `g2.setColor(new Color(180, 180, 190));`.
- **Nếu cô yêu cầu sửa:** Đổi màu ở dòng 438.

### Câu 24. Độ dày hoặc độ thưa của lưới sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `drawGrid(...)`, dòng 423.
- **Nói khi mở code:** `size` là kích thước sàn, `step` là bước giữa các đường lưới.
- **Dòng quan trọng:** Dòng 423: `int size = 40, step = 5;`.
- **Nếu cô yêu cầu sửa:** Giảm `step` để lưới dày hơn, tăng `step` để lưới thưa hơn.

### Câu 25. Màu thân robot đổi ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 188-195.
- **Nói khi mở code:** Mảng `tubeColors` quy định màu các link robot.
- **Dòng quan trọng:** Dòng 188-195.
- **Nếu cô yêu cầu sửa:** Đổi các `new Color(...)` trong mảng `tubeColors`.

### Câu 26. Độ dày thân robot sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 187.
- **Nói khi mở code:** Mảng `tubeWidths` quy định độ dày các link.
- **Dòng quan trọng:** Dòng 187: `int[] tubeWidths = { 7, 6, 5, 4, 3, 2 };`.
- **Nếu cô yêu cầu sửa:** Tăng các số để link to hơn, giảm các số để link nhỏ hơn.

### Câu 27. Link robot được vẽ bằng class nào?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, class `TubeSegment`, dòng 276-313.
- **Nói khi mở code:** `TubeSegment` đại diện cho một thanh nối/link của robot.
- **Dòng quan trọng:** Dòng 294-312 là hàm `draw(...)`.
- **Nếu cô yêu cầu sửa:** Muốn đổi kiểu vẽ link thì sửa trong `TubeSegment.draw(...)`.

### Câu 28. Link robot được vẽ bằng lệnh Java nào?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, class `TubeSegment`, dòng 294-312.
- **Nói khi mở code:** Link được vẽ bằng `g2.drawLine(...)` với `BasicStroke` để có độ dày.
- **Dòng quan trọng:** Dòng 306-311.
- **Nếu cô yêu cầu sửa:** Muốn link đậm hơn thì tăng `tw` hoặc stroke.

### Câu 29. Khớp tròn được vẽ ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, class `JointSphere`, dòng 315-347.
- **Nói khi mở code:** `JointSphere` vẽ các khớp robot bằng hình tròn.
- **Dòng quan trọng:** Dòng 341-345 dùng `fillOval` và `drawOval`.
- **Nếu cô yêu cầu sửa:** Muốn đổi màu khớp thì sửa màu khi tạo `JointSphere`.

### Câu 30. Kích thước khớp tròn sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, class `JointSphere`, dòng 332-339.
- **Nói khi mở code:** Bán kính khớp được tính vào biến `jr`.
- **Dòng quan trọng:** Dòng 337 có hệ số `0.6`.
- **Nếu cô yêu cầu sửa:** Tăng `0.6` lên `0.9` để khớp to hơn.

### Câu 31. Kẹp robot được vẽ ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, class `GripperDrawable`, dòng 349-409.
- **Nói khi mở code:** `GripperDrawable` vẽ thanh ngang và hai ngón kẹp.
- **Dòng quan trọng:** Dòng 368-408 là hàm `draw(...)`.
- **Nếu cô yêu cầu sửa:** Muốn đổi hình dạng kẹp thì sửa trong class này.

### Câu 32. Màu kẹp robot sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, class `GripperDrawable`, dòng 393-400.
- **Nói khi mở code:** Thanh ngang kẹp có màu xám, ngón kẹp có màu cam.
- **Dòng quan trọng:** Dòng 394 và dòng 399.
- **Nếu cô yêu cầu sửa:** Đổi `Color.ORANGE` ở dòng 399 thành màu khác.

### Câu 33. Độ mở kẹp sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, class `GripperDrawable`, dòng 373-378.
- **Nói khi mở code:** `wOpening` quy định khoảng cách giữa hai ngón kẹp.
- **Dòng quan trọng:** Dòng 374.
- **Nếu cô yêu cầu sửa:** Đổi `1.5` thành `2.5` nếu muốn kẹp mở rộng hơn.

### Câu 34. Độ dài ngón kẹp sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 221-225.
- **Nói khi mở code:** Tham số cuối của `GripperDrawable` là độ dài ngón kẹp hiển thị.
- **Dòng quan trọng:** Dòng 225: `new GripperDrawable(T_end, pTCP, 3.5)`.
- **Nếu cô yêu cầu sửa:** Đổi `3.5` thành `5.0` nếu muốn ngón kẹp dài hơn.

### Câu 35. Vết quỹ đạo được vẽ ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `drawTrail(...)`, dòng 449-455.
- **Nói khi mở code:** Hàm này nối các điểm trail thành đường quỹ đạo.
- **Dòng quan trọng:** Dòng 450 đặt màu; dòng 452-453 vẽ line.
- **Nếu cô yêu cầu sửa:** Đổi màu ở dòng 450 hoặc thêm `BasicStroke` để đường dày hơn.

### Câu 36. Vết quỹ đạo được lưu thêm điểm ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 162-176.
- **Nói khi mở code:** Khi bật trail, vị trí đầu kẹp hiện tại được thêm vào danh sách `trail`.
- **Dòng quan trọng:** Dòng 170 là ngưỡng thêm điểm; dòng 172 là số điểm tối đa.
- **Nếu cô yêu cầu sửa:** Đổi `dist > 1.0` thành `0.5` để mịn hơn; đổi `500` thành `1000` để dài hơn.

### Câu 37. Workspace được vẽ ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `drawWorkspace(...)`, dòng 528-549.
- **Nói khi mở code:** Workspace được vẽ bằng các điểm và một số đường nối.
- **Dòng quan trọng:** Dòng 531, 542 đặt màu; dòng 546 vẽ điểm.
- **Nếu cô yêu cầu sửa:** Đổi màu ở dòng 531/542 hoặc đổi kích thước điểm ở dòng 546.

### Câu 38. Điểm click màu đỏ được vẽ ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 235-238.
- **Nói khi mở code:** Nếu có `clickTarget`, panel vẽ một chấm đánh dấu vị trí click.
- **Dòng quan trọng:** Dòng 237 đặt màu; dòng 238 vẽ oval.
- **Nếu cô yêu cầu sửa:** Đổi màu ở dòng 237; đổi kích thước `8,8` thành `14,14`.

### Câu 39. Chữ tiêu đề trong vùng 3D vẽ ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 241-242.
- **Nói khi mở code:** Dòng chữ góc trên trái được vẽ bằng `drawString`.
- **Dòng quan trọng:** Dòng 242.
- **Nếu cô yêu cầu sửa:** Đổi chuỗi trong `drawString` hoặc thêm `setFont` trước dòng 242.

### Câu 40. Muốn đổi toàn bộ tỉ lệ hiển thị robot thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, field dòng 13.
- **Nói khi mở code:** Biến `scale` quy định tỉ lệ hiển thị ban đầu.
- **Dòng quan trọng:** Dòng 13: `double camAz = -30, camEl = 25, scale = 25.0;`.
- **Nếu cô yêu cầu sửa:** Tăng `scale` để robot to hơn, giảm `scale` để robot nhỏ hơn.

---

## D. Xử lý chuột, camera và zoom

### Câu 41. Góc camera được lưu ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, field dòng 13.
- **Nói khi mở code:** `camAz` là góc quay ngang, `camEl` là góc nâng, `scale` là tỉ lệ zoom.
- **Dòng quan trọng:** Dòng 13.
- **Nếu cô yêu cầu sửa:** Đổi giá trị khởi tạo nếu muốn góc nhìn mặc định khác.

### Câu 42. Bắt sự kiện nhấn chuột ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `mousePressed(...)`, dòng 31-34.
- **Nói khi mở code:** Khi nhấn chuột, chương trình lưu lại vị trí chuột cũ.
- **Dòng quan trọng:** Dòng 32-34.
- **Nếu cô yêu cầu sửa:** Nếu cần thêm hành vi khi bắt đầu kéo chuột thì sửa hàm này.

### Câu 43. Kéo chuột để xoay camera xử lý ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `mouseDragged(...)`, dòng 60-69.
- **Nói khi mở code:** Kéo chuột trái làm đổi `camAz`, `camEl`, sau đó repaint.
- **Dòng quan trọng:** Dòng 63-64 đổi góc; dòng 68 repaint.
- **Nếu cô yêu cầu sửa:** Tăng hệ số `0.5` nếu muốn xoay nhanh hơn.

### Câu 44. Giới hạn góc camera lên/xuống ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `mouseDragged(...)`, dòng 64.
- **Nói khi mở code:** `camEl` bị giới hạn trong khoảng `-85` đến `85` để tránh lật hình.
- **Dòng quan trọng:** Dòng 64: `Math.max(-85, Math.min(85, ...))`.
- **Nếu cô yêu cầu sửa:** Đổi `-85`, `85` thành khoảng khác.

### Câu 45. Lăn chuột zoom xử lý ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `mouseWheelMoved(...)`, dòng 77-80.
- **Nói khi mở code:** Lăn chuột làm thay đổi `scale`, rồi gọi `repaint()`.
- **Dòng quan trọng:** Dòng 78 thay đổi scale; dòng 79 repaint.
- **Nếu cô yêu cầu sửa:** Đổi hệ số `0.1` thành `0.2` để zoom nhanh hơn.

### Câu 46. Click chuột phải di chuyển robot xử lý ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `mouseClicked(...)`, dòng 38-44.
- **Nói khi mở code:** Nếu bật fixed-height mode và click chuột phải, chương trình gọi `moveToScreenPoint`.
- **Dòng quan trọng:** Dòng 41-42.
- **Nếu cô yêu cầu sửa:** Muốn đổi sang click chuột trái thì sửa điều kiện ở dòng 41.

### Câu 47. Chuyển điểm màn hình sang tọa độ robot ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `moveToScreenPoint(...)`, dòng 86-124.
- **Nói khi mở code:** Hàm này đổi điểm 2D trên màn hình thành tọa độ 3D với Z cố định.
- **Dòng quan trọng:** Dòng 87-94 lấy tâm và scale; dòng 111-117 gọi IK.
- **Nếu cô yêu cầu sửa:** Muốn click chính xác hơn thì tăng số vòng lặp ở dòng 101.

### Câu 48. Vì sao click-to-move cần Z cố định?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `moveToScreenPoint(...)`, dòng 87-111.
- **Nói khi mở code:** Điểm click trên màn hình chỉ là 2D, nên cần thêm chiều cao Z để suy ra điểm 3D.
- **Dòng quan trọng:** Dòng 88 lấy `fixedZ`; dòng 111 tạo `targetPos`.
- **Nếu cô yêu cầu sửa:** Muốn đổi giá trị Z thì sửa spinner trong `MainFrame`.

### Câu 49. Marker click được lưu bằng biến nào?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, field dòng 16; vẽ ở dòng 235-238.
- **Nói khi mở code:** `clickTarget` lưu điểm vừa click để vẽ marker trên màn hình.
- **Dòng quan trọng:** Dòng 16 và dòng 235-238.
- **Nếu cô yêu cầu sửa:** Muốn bỏ marker thì không vẽ khối `if (clickTarget != null)`.

### Câu 50. Đăng ký listener chuột cho panel 3D ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, constructor dòng 22-28.
- **Nói khi mở code:** Panel phải đăng ký listener thì mới nhận được sự kiện chuột.
- **Dòng quan trọng:** Dòng 25-27.
- **Nếu cô yêu cầu sửa:** Nếu bỏ một dòng listener thì sự kiện tương ứng sẽ không hoạt động.

---

## E. Layout, component và sự kiện Swing

### Câu 51. Layout chính của cửa sổ đặt ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, constructor `MainFrame()`, dòng 97.
- **Nói khi mở code:** Cửa sổ dùng `BorderLayout`.
- **Dòng quan trọng:** Dòng 97: `setLayout(new BorderLayout());`.
- **Nếu cô yêu cầu sửa:** Muốn đổi cách sắp xếp toàn cửa sổ thì sửa layout này.

### Câu 52. Control panel bên phải dùng layout nào?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildControlPanel()`, dòng 117-122.
- **Nói khi mở code:** `controlPanel` dùng `BorderLayout` và nằm bên phải.
- **Dòng quan trọng:** Dòng 119.
- **Nếu cô yêu cầu sửa:** Muốn đổi cách sắp xếp trong panel phải thì sửa layout ở dòng 119.

### Câu 53. Tab điều khiển và quỹ đạo tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildControlPanel()`, dòng 235-241.
- **Nói khi mở code:** `JTabbedPane` chia panel phải thành tab điều khiển và tab quỹ đạo.
- **Dòng quan trọng:** Dòng 237 và 239.
- **Nếu cô yêu cầu sửa:** Đổi tên tab trong `mainTabs.addTab(...)`.

### Câu 54. Vì sao dùng `JScrollPane` cho tab điều khiển?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildControlPanel()`, dòng 235-237.
- **Nói khi mở code:** Panel điều khiển dài, nên bọc bằng `JScrollPane` để có thể cuộn.
- **Dòng quan trọng:** Dòng 235 tạo `JScrollPane`.
- **Nếu cô yêu cầu sửa:** Nếu bỏ scroll pane, khi cửa sổ nhỏ có thể mất component.

### Câu 55. Slider góc khớp được tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildControlPanel()`, dòng 128-146.
- **Nói khi mở code:** Vòng lặp tạo slider cho từng khớp.
- **Dòng quan trọng:** Dòng 135 tạo `JSlider`; dòng 138 add `ChangeListener`.
- **Nếu cô yêu cầu sửa:** Muốn đổi min/max thì sửa `JOINT_MIN`, `JOINT_MAX` trong `Kinematics`.

### Câu 56. Label hiển thị góc khớp tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildControlPanel()`, dòng 140-145.
- **Nói khi mở code:** Mỗi slider có một label hiển thị giá trị góc.
- **Dòng quan trọng:** Dòng 140 tạo label; dòng 145 add vào panel.
- **Nếu cô yêu cầu sửa:** Muốn label rộng hơn thì sửa `setPreferredSize`.

### Câu 57. Ô nhập tọa độ X/Y/Z tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 42-44; add vào UI dòng 159-175.
- **Nói khi mở code:** Ba `JTextField` cho phép nhập tọa độ chính xác.
- **Dòng quan trọng:** Dòng 42-44 và 159-175.
- **Nếu cô yêu cầu sửa:** Muốn đổi giá trị mặc định thì sửa chuỗi trong `new JTextField(...)`.

### Câu 58. Slider tọa độ X/Y/Z tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 45-47; add listener dòng 182-184.
- **Nói khi mở code:** Ba slider này dùng để thử nhanh tọa độ.
- **Dòng quan trọng:** Dòng 45-47 tạo slider; dòng 182-184 add listener.
- **Nếu cô yêu cầu sửa:** Muốn mở rộng vùng kéo thì sửa min/max trong `new JSlider(...)`.

### Câu 59. Nút “Đến tọa độ” tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 48; add vào UI dòng 186-190.
- **Nói khi mở code:** Nút này gọi hàm đi đến tọa độ khi người dùng bấm.
- **Dòng quan trọng:** Dòng 48 tạo nút; dòng 193 add listener.
- **Nếu cô yêu cầu sửa:** Đổi chữ nút ở dòng 48.

### Câu 60. Bắt Enter trong ô tọa độ ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildControlPanel()`, dòng 195-198.
- **Nói khi mở code:** Ba ô X/Y/Z cùng gắn listener để nhận Enter.
- **Dòng quan trọng:** Dòng 195-198.
- **Nếu cô yêu cầu sửa:** Muốn chỉ ô Z nhận Enter thì bỏ listener của `txX`, `txY`.

### Câu 61. Checkbox click mode tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 69; UI dòng 200-220.
- **Nói khi mở code:** Checkbox này bật/tắt chế độ click đến tọa độ với Z cố định.
- **Dòng quan trọng:** Dòng 69 và 200-220.
- **Nếu cô yêu cầu sửa:** Đổi tên checkbox ở dòng 69.

### Câu 62. Spinner chiều cao Z cố định tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 70; UI dòng 208-212.
- **Nói khi mở code:** `JSpinner` cho phép chọn giá trị Z cố định.
- **Dòng quan trọng:** Dòng 70.
- **Nếu cô yêu cầu sửa:** Đổi min/max/bước trong `SpinnerNumberModel`.

### Câu 63. Checkbox hiện lưới và trail nằm ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 30-31.
- **Nói khi mở code:** Hai checkbox này lưu trạng thái hiện lưới và vết quỹ đạo.
- **Dòng quan trọng:** Dòng 30-31.
- **Nếu cô yêu cầu sửa:** Đổi giá trị `true/false` để đổi trạng thái mặc định.

### Câu 64. Menu hiển thị tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildMenuBar()`, dòng 471-514.
- **Nói khi mở code:** Menu này cho phép bật/tắt lưới, trail, workspace và đổi góc nhìn.
- **Dòng quan trọng:** Dòng 473, 479, 485, 501, 505.
- **Nếu cô yêu cầu sửa:** Muốn thêm lựa chọn hiển thị thì thêm menu item trong đoạn này.

### Câu 65. Xử lý menu Top View ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, class `MenuItemListener`, dòng 548-563.
- **Nói khi mở code:** Khi chọn Top View, code đổi `camAz`, `camEl` rồi repaint.
- **Dòng quan trọng:** Dòng 552-556.
- **Nếu cô yêu cầu sửa:** Đổi giá trị `camAz`, `camEl` nếu muốn góc top view khác.

### Câu 66. Xử lý menu 3D Perspective ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, class `MenuItemListener`, dòng 557-560.
- **Nói khi mở code:** Menu này đưa camera về góc nhìn phối cảnh mặc định.
- **Dòng quan trọng:** Dòng 558-559.
- **Nếu cô yêu cầu sửa:** Đổi `-30`, `25` thành góc nhìn mong muốn.

### Câu 67. Nút đóng/mở kẹp tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildTopPanel()`, dòng 386-391.
- **Nói khi mở code:** Nút này đảo biến `isGripped`, repaint panel và hiện trạng thái.
- **Dòng quan trọng:** Dòng 386 tạo nút; dòng 388 đảo trạng thái; dòng 389 repaint.
- **Nếu cô yêu cầu sửa:** Đổi chữ nút ở dòng 386.

### Câu 68. Thanh tốc độ tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 72-73; UI dòng 394-400.
- **Nói khi mở code:** `speedSlider` điều chỉnh tốc độ di chuyển.
- **Dòng quan trọng:** Dòng 72 tạo slider; dòng 398 cập nhật label.
- **Nếu cô yêu cầu sửa:** Đổi max `120` thành `180` nếu muốn nhanh hơn.

### Câu 69. Ô COM và nút kết nối tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildTopPanel()`, dòng 402-441.
- **Nói khi mở code:** Top panel có combo COM, nút refresh và nút kết nối.
- **Dòng quan trọng:** Dòng 404 tạo combo; dòng 410 tạo nút kết nối; dòng 429 tạo refresh.
- **Nếu cô yêu cầu sửa:** Muốn đổi baudrate thì sửa dòng 415.

### Câu 70. Menu item xử lý bằng `actionCommand` ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, class `MenuItemListener`, dòng 545-564.
- **Nói khi mở code:** Listener dùng `switch(e.getActionCommand())` để xử lý menu.
- **Dòng quan trọng:** Dòng 549-562.
- **Nếu cô yêu cầu sửa:** Thêm case mới nếu thêm menu item mới.

---

## F. Event, repaint, timer và thread

### Câu 71. Xử lý khi bấm nút nằm ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `actionPerformed(...)`, dòng 695-719.
- **Nói khi mở code:** Hàm này xử lý các nút và checkbox dùng chung `ActionListener`.
- **Dòng quan trọng:** Dòng 708-710 xử lý `btnGoto`; dòng 710-714 xử lý lấy tọa độ.
- **Nếu cô yêu cầu sửa:** Đổi hành động nút trong nhánh `if/else` tương ứng.

### Câu 72. Xử lý khi kéo slider nằm ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `stateChanged(...)`, dòng 722-763.
- **Nói khi mở code:** Hàm này xử lý `ChangeEvent` của slider góc và slider tọa độ.
- **Dòng quan trọng:** Dòng 723-739 xử lý slider góc; dòng 741-763 xử lý slider X/Y/Z.
- **Nếu cô yêu cầu sửa:** Thêm logic khi slider đổi trong hàm này.

### Câu 73. Khi kéo slider góc thì robot vẽ lại ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `stateChanged(...)`, dòng 723-739.
- **Nói khi mở code:** Slider đổi góc, cập nhật `angles`, rồi gọi `updateArm()`.
- **Dòng quan trọng:** Dòng 726 cập nhật góc; dòng 737 gọi `updateArm()`.
- **Nếu cô yêu cầu sửa:** Muốn thêm cảnh báo khi kéo slider thì thêm trong đoạn này.

### Câu 74. Hàm `updateArm()` làm gì?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `updateArm()`, dòng 766-794.
- **Nói khi mở code:** Hàm này cập nhật tọa độ đầu kẹp, đồng bộ UI khi cần và repaint panel 3D.
- **Dòng quan trọng:** Dòng 771 cập nhật label; dòng 794 repaint.
- **Nếu cô yêu cầu sửa:** Muốn đổi format tọa độ thì sửa dòng 771.

### Câu 75. `repaint()` được gọi ở đâu khi cập nhật robot?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `updateArm()`, dòng 794.
- **Nói khi mở code:** Sau khi cập nhật dữ liệu, `armPanel.repaint()` yêu cầu Swing vẽ lại 3D.
- **Dòng quan trọng:** Dòng 794.
- **Nếu cô yêu cầu sửa:** Nếu thêm thay đổi ảnh hưởng hình vẽ, cần gọi repaint sau thay đổi đó.

### Câu 76. Robot di chuyển mượt bằng timer ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `startMotionTimer()`, dòng 568-572; hàm `tickMotion()`, dòng 579-617.
- **Nói khi mở code:** Swing Timer gọi `tickMotion()` để nội suy góc theo từng bước.
- **Dòng quan trọng:** Dòng 571 tạo timer; dòng 599 cập nhật góc; dòng 615 update UI.
- **Nếu cô yêu cầu sửa:** Muốn mượt hơn thì giảm `MOTION_DT_MS` ở dòng 74.

### Câu 77. Chu kỳ timer sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 74.
- **Nói khi mở code:** `MOTION_DT_MS` là thời gian mỗi lần cập nhật chuyển động.
- **Dòng quan trọng:** Dòng 74: `private static final int MOTION_DT_MS = 30;`.
- **Nếu cô yêu cầu sửa:** Đổi `30` thành `15` nếu muốn cập nhật nhanh hơn.

### Câu 78. Vì sao remove listener khi set slider bằng code?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `tickMotion()`, dòng 609-613.
- **Nói khi mở code:** Khi chương trình tự set slider, tạm remove listener để tránh lặp sự kiện.
- **Dòng quan trọng:** Dòng 610 remove; dòng 611 set; dòng 612 add lại.
- **Nếu cô yêu cầu sửa:** Không nên bỏ đoạn này nếu slider được cập nhật từ code.

### Câu 79. Reset robot nằm ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `resetAngles()`, dòng 797-801.
- **Nói khi mở code:** Reset xóa trail và đặt target góc về 0.
- **Dòng quan trọng:** Dòng 798 xóa trail; dòng 799 tạo mảng zero.
- **Nếu cô yêu cầu sửa:** Đổi `new double[NUM_JOINTS]` thành `{0, -45, 20, 60, 0}` nếu muốn reset về pose khác.

### Câu 80. Bắt lỗi nhập sai tọa độ ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `gotoCoordinate()`, dòng 670-692.
- **Nói khi mở code:** Hàm đọc text, parse sang `double`, nếu sai thì bắt `NumberFormatException`.
- **Dòng quan trọng:** Dòng 672-674 parse; dòng 689-691 bắt lỗi.
- **Nếu cô yêu cầu sửa:** Muốn hiện popup thì dùng `JOptionPane.showMessageDialog(...)` trong catch.

### Câu 81. Vì sao không cho Z âm?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `gotoCoordinate()`, dòng 676-679.
- **Nói khi mở code:** Z âm được xem là đi xuống dưới mặt sàn, nên chương trình báo lỗi.
- **Dòng quan trọng:** Dòng 676-679.
- **Nếu cô yêu cầu sửa:** Muốn cho phép Z âm thì bỏ hoặc đổi điều kiện này.

### Câu 82. Thread quét workspace tạo ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `runWorkspaceExploration()`, dòng 1248-1303.
- **Nói khi mở code:** Workspace tính nhiều điểm nên chạy trên thread riêng để không làm đơ giao diện.
- **Dòng quan trọng:** Dòng 1252 tạo `Thread`; dòng 1302 start.
- **Nếu cô yêu cầu sửa:** Tác vụ tính toán nặng khác cũng nên cân nhắc dùng thread riêng.

### Câu 83. Cập nhật Swing từ thread phụ an toàn ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `runWorkspaceExploration()`, dòng 1280-1284 và 1294-1298.
- **Nói khi mở code:** Thread phụ dùng `SwingUtilities.invokeLater` để cập nhật UI trên EDT.
- **Dòng quan trọng:** Dòng 1281 và 1294.
- **Nếu cô yêu cầu sửa:** Nếu cập nhật component Swing từ thread phụ thì nên bọc trong `invokeLater`.

### Câu 84. Status của workspace được vẽ ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, hàm `paintComponent(...)`, dòng 147-150.
- **Nói khi mở code:** Nếu `workspaceStatus` không rỗng, panel vẽ chữ trạng thái lên vùng 3D.
- **Dòng quan trọng:** Dòng 148 set font; dòng 149 set màu; dòng 150 draw string.
- **Nếu cô yêu cầu sửa:** Muốn đổi vị trí text thì sửa tọa độ ở dòng 150.

### Câu 85. Trạng thái OK/lỗi của tọa độ hiện ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field dòng 50; hàm `setGotoStatus(...)`, dòng 665-668.
- **Nói khi mở code:** `gotoStatus` là label hiện trạng thái OK hoặc lỗi.
- **Dòng quan trọng:** Dòng 665-668.
- **Nếu cô yêu cầu sửa:** Muốn đổi màu mặc định thì sửa các chỗ gọi `setGotoStatus`.

---

## G. Các câu mở code khi cô yêu cầu sửa trực tiếp

### Câu 86. Cô muốn đổi tên cửa sổ thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, constructor `MainFrame()`, dòng 105.
- **Nói khi mở code:** Tiêu đề cửa sổ được đặt bằng `setTitle`.
- **Dòng quan trọng:** Dòng 105.
- **Nếu cô yêu cầu sửa:** Đổi chuỗi trong `setTitle(...)`.

### Câu 87. Cô muốn đổi Look and Feel thì sửa ở đâu?
- **Vị trí mở code:** package `app`, class `App`, file `src/app/App.java`, hàm `main(...)`, dòng 23-27.
- **Nói khi mở code:** Chương trình đặt giao diện theo Look and Feel của hệ điều hành.
- **Dòng quan trọng:** Dòng 24.
- **Nếu cô yêu cầu sửa:** Đổi hoặc bỏ `UIManager.setLookAndFeel(...)`.

### Câu 88. Cô muốn đổi tên các khớp trên slider thì sửa ở đâu?
- **Vị trí mở code:** package `kinematics`, class `Kinematics`, file `src/kinematics/Kinematics.java`, dòng 14.
- **Nói khi mở code:** Tên khớp hiển thị lấy từ mảng `JOINT_NAMES`.
- **Dòng quan trọng:** Dòng 14.
- **Nếu cô yêu cầu sửa:** Đổi `"Khớp 1"` thành `"Base"` hoặc tên mong muốn.

### Câu 89. Cô muốn đổi giới hạn góc khớp thì sửa ở đâu?
- **Vị trí mở code:** package `kinematics`, class `Kinematics`, file `src/kinematics/Kinematics.java`, dòng 15-16.
- **Nói khi mở code:** Giới hạn slider và nghiệm hợp lệ lấy từ `JOINT_MIN`, `JOINT_MAX`.
- **Dòng quan trọng:** Dòng 15-16.
- **Nếu cô yêu cầu sửa:** Đổi giá trị min/max của từng khớp trong hai mảng này.

### Câu 90. Cô muốn đổi chiều dài các link robot thì sửa ở đâu?
- **Vị trí mở code:** package `kinematics`, class `Kinematics`, file `src/kinematics/Kinematics.java`, dòng 8-13.
- **Nói khi mở code:** Các hằng `L1` đến `L6` là thông số kích thước robot.
- **Dòng quan trọng:** Dòng 8-13.
- **Nếu cô yêu cầu sửa:** Đổi giá trị các hằng theo kích thước robot thật, rồi kiểm tra lại FK/IK.

### Câu 91. Cô muốn đổi tốc độ gửi/di chuyển thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, field `speedSlider`, dòng 72.
- **Nói khi mở code:** Slider tốc độ quy định tốc độ nội suy góc.
- **Dòng quan trọng:** Dòng 72.
- **Nếu cô yêu cầu sửa:** Đổi max `120` thành `180` hoặc `240`.

### Câu 92. Cô muốn đổi format UART thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `sendJointsToUart()`, dòng 623-654.
- **Nói khi mở code:** Hàm này ghép chuỗi góc khớp và gửi qua UART.
- **Dòng quan trọng:** Dòng 637 tạo tiền tố `S:`; dòng 646 thêm `\n`.
- **Nếu cô yêu cầu sửa:** Đổi tiền tố hoặc thêm dữ liệu trước dòng 646.

### Câu 93. Cô muốn gửi thêm trạng thái kẹp qua UART thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `sendJointsToUart()`, dòng 637-646.
- **Nói khi mở code:** Sau khi ghép 5 góc, có thể ghép thêm trạng thái kẹp.
- **Dòng quan trọng:** Trước dòng 646.
- **Nếu cô yêu cầu sửa:** Thêm `sb.append(",").append(isGripped ? 1 : 0);` trước `sb.append("\n");`.

### Câu 94. Cô muốn đổi baudrate thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildTopPanel()`, dòng 411-416.
- **Nói khi mở code:** Khi kết nối COM, chương trình truyền baudrate `115200`.
- **Dòng quan trọng:** Dòng 415.
- **Nếu cô yêu cầu sửa:** Đổi `115200` thành baudrate khác và STM32 cũng phải đổi giống vậy.

### Câu 95. Cô muốn đổi nút refresh COM thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildTopPanel()`, dòng 429-437.
- **Nói khi mở code:** Nút refresh xóa danh sách COM và nạp lại từ `UartManager`.
- **Dòng quan trọng:** Dòng 429 tạo nút; dòng 433-436 nạp lại danh sách.
- **Nếu cô yêu cầu sửa:** Muốn đổi chữ nút thì sửa dòng 429.

### Câu 96. Cô muốn đổi cách lấy danh sách COM thì sửa ở đâu?
- **Vị trí mở code:** package `comm`, class `UartManager`, file `src/comm/UartManager.java`, hàm `getAvailablePorts()`, dòng 14-20.
- **Nói khi mở code:** Hàm này dùng `SerialPort.getCommPorts()` để lấy các cổng hiện có.
- **Dòng quan trọng:** Dòng 16-18.
- **Nếu cô yêu cầu sửa:** Muốn hiện tên mô tả thì thay `getSystemPortName()` bằng `getDescriptivePortName()`.

### Câu 97. Cô muốn đổi cấu hình serial 8N1 thì sửa ở đâu?
- **Vị trí mở code:** package `comm`, class `UartManager`, file `src/comm/UartManager.java`, hàm `connect(...)`, dòng 23-35.
- **Nói khi mở code:** Serial port được cấu hình baudrate, 8 data bit, 1 stop bit, no parity.
- **Dòng quan trọng:** Dòng 25: `port.setComPortParameters(baudRate, 8, 1, 0);`.
- **Nếu cô yêu cầu sửa:** Đổi các tham số `8, 1, 0` nếu firmware yêu cầu cấu hình khác.

### Câu 98. Cô muốn đổi cách gửi byte qua UART thì sửa ở đâu?
- **Vị trí mở code:** package `comm`, class `UartManager`, file `src/comm/UartManager.java`, hàm `sendData(String data)`, dòng 44-51.
- **Nói khi mở code:** Hàm này đổi chuỗi thành byte và ghi ra serial port.
- **Dòng quan trọng:** Dòng 46-47.
- **Nếu cô yêu cầu sửa:** Muốn đổi encoding thì sửa `data.getBytes(...)`.

### Câu 99. Cô muốn đổi thông báo kết nối COM thì sửa ở đâu?
- **Vị trí mở code:** package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, hàm `buildTopPanel()`, dòng 411-426.
- **Nói khi mở code:** Khi kết nối thành công/thất bại, chương trình gọi `setGotoStatus`.
- **Dòng quan trọng:** Dòng 417, 419, 425.
- **Nếu cô yêu cầu sửa:** Đổi chuỗi thông báo trong các dòng này.

### Câu 100. Nếu cô hỏi “muốn sửa giao diện 3D thì em tìm file nào trước?” trả lời và mở code ở đâu?
- **Vị trí mở code:** package `gui`, class `ArmPanel`, file `src/gui/ArmPanel.java`, dòng 9-10; package `gui`, class `MainFrame`, file `src/gui/MainFrame.java`, dòng 94-110.
- **Nói khi mở code:** Nếu sửa hình vẽ 3D thì mở `ArmPanel.java`; nếu sửa nút, slider, tab và bố cục thì mở `MainFrame.java`.
- **Dòng quan trọng:** `ArmPanel.java` dòng 9-10; `MainFrame.java` dòng 95-102.
- **Nếu cô yêu cầu sửa:** Xác định đối tượng cần sửa là hình vẽ hay component giao diện, rồi mở đúng file tương ứng.
