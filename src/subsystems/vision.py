import wpilib
import ntcore
import math
import commands2

class VisionSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        
        # 1. Initialize NetworkTables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.limelight_table = self.inst.getTable("limelight")
        
        # 2. Start Data Logging (For AdvantageScope)
        wpilib.DataLogManager.start()
        
        # 3. Configuration (Update after your robot is built!)
        self.LIMELIGHT_HEIGHT = 18.0   # Floor to lens center (inches)
        self.MOUNT_ANGLE_DEG = -22.0   # 22 degrees DOWN from horizontal
        
        # 4. 2026 REBUILT AprilTag Heights (Update from 2026 Game Manual)
        self.TAG_HEIGHT_MAP = {
            1: 27.0, 2: 27.0,   # Tower Rungs
            3: 63.0, 4: 63.0    # Scoring Targets
        }

        # 5. Logging Throttle (Prevents terminal spam)
        self.log_counter = 0

    def get_distance_to_tag(self) -> float:
        tv = self.limelight_table.getNumber("tv", 0)    
        tid = int(self.limelight_table.getNumber("tid", -1))
        ty = self.limelight_table.getNumber("ty", 0.0)

        if tv < 1.0 or tid not in self.TAG_HEIGHT_MAP:
            return -1.0

        target_height = self.TAG_HEIGHT_MAP[tid]
        angle_to_target_rad = math.radians(self.MOUNT_ANGLE_DEG + ty)
        height_difference = target_height - self.LIMELIGHT_HEIGHT
        
        try:
            return abs(height_difference / math.tan(angle_to_target_rad))
        except ZeroDivisionError:
            return -1.0

    def periodic(self):
        """Runs every 20ms."""
        dist = self.get_distance_to_tag()
        tag_id = int(self.limelight_table.getNumber("tid", -1))

        # Update SmartDashboard
        wpilib.SmartDashboard.putNumber("Vision/Distance_In", dist)
        wpilib.SmartDashboard.putNumber("Vision/Active_Tag_ID", tag_id)

        # --- TERMINAL LOGGING ---
        self.log_counter += 1
        if self.log_counter >= 25: # Only print every ~0.5 seconds
            if dist > 0:
                print(f"[Vision] Tag {tag_id} detected! Distance: {dist:.2f} inches")
            else:
                print("[Vision] No valid target in view.")
            self.log_counter = 0

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.vision = VisionSubsystem()
        print("--- 2026 REBUILT Robot System Online ---")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
