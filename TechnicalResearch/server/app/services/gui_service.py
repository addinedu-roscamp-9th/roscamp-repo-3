"""
GUI 서비스 로직
"""

from app.models.gui_model import GuiData
from app.services.ros2_bridge import get_ros2_bridge


def handle_gui_command(data: GuiData):
    """GUI 명령 처리"""

    # ROS2 브릿지 가져오기
    ros2_bridge = get_ros2_bridge()

    # 이동 명령
    if data.command == "move":
        success = ros2_bridge.move_to_location(data.destination)
        return {
            "status": "success" if success else "error",
            "message": f"Moving to {data.destination}" if success else "Failed to move",
            "destination": data.destination,
        }

    # 정지 명령
    elif data.command == "stop":
        ros2_bridge.emergency_stop()
        return {"status": "success", "message": "Robot stopped"}

    # 물건 가져오기
    elif data.command == "bring_item":
        # 1단계: pickup_zone으로 이동
        ros2_bridge.move_to_location("pickup_zone")
        # 2단계: 목적지로 이동
        ros2_bridge.move_to_location(data.destination)

        return {
            "status": "success",
            "message": f"Bringing {data.item} to {data.destination}",
        }

    # 물건 갖다놓기
    elif data.command == "put_item":
        # 1단계: 현재 방에서 물건 싣기
        ros2_bridge.move_to_location(data.from_room)
        # 2단계: pickup_zone으로 이동
        ros2_bridge.move_to_location("pickup_zone")

        return {
            "status": "success",
            "message": f"Putting item from {data.from_room} to pickup zone",
        }

    # 스케줄 실행
    elif data.command == "execute_schedule":
        for task in data.tasks:
            ros2_bridge.move_to_location(task)

        return {
            "status": "success",
            "message": f"Executing schedule: {data.schedule_name}",
        }

    return {"status": "error", "message": "Unknown command"}
