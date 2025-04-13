from tello_msgs.msg import FlipControl
from std_msgs.msg import Empty

class EmotionHandler:
    def __init__(self, controller):
        self.controller = controller
        self.timer = controller.create_timer(controller.calltime, self.react)

    def react(self):
        c = self.controller
        if c.emotionactive and c.latest_emotion:
            c.get_logger().info(f"Latest Emotion: {c.latest_emotion}")

            # Happy
            if c.happyperforming or (c.notperformed and c.latest_emotion == "Happy"):
                c.get_logger().info("Happy Front FLip!")
                msg = FlipControl()
                msg.flip_backward = True
                c._flip_control_pub.publish(msg)
                c.notperformed = False
                c.happyclean = True
                c.happyperforming = False
                return

            if c.happyclean:
                msg = FlipControl()
                c._flip_control_pub.publish(msg)  # All flips are False by default
                c.get_logger().info("Happy Clean!")
                c.happyclean = False
                c.happyperf = True
                return

            if c.happyperf:
                msg = FlipControl()
                msg.flip_forward = True
                c._flip_control_pub.publish(msg)
                c.get_logger().info("Happy Back Flip!")
                c.happyperf = False

            # Angry
            if c.angryperforming or (c.notperformed and c.latest_emotion == "Angry"):
                c.key_pressed["th"] = -c.speed
                c.get_logger().info("Angry Movement 1!")
                c.notperformed = False
                c.angryclean = True
                c.angryperforming = False
                return

            if c.angryclean:
                c.key_pressed["th"] = 0.0
                c.get_logger().info("Angry Clean!")
                c.angryclean = False
                c.angryperf = True
                return

            if c.angryperf and c.count < 4:
                c.count += 1
                c.key_pressed["cw"] = c.speed
                c.angryperf2 = True
                c.get_logger().info("Angry Movement 2!")
                return
            elif c.angryperf:
                c.key_pressed["cw"] = 0.0
                c.count = 0
                c.angryperf = False

            if c.angryperf2:
                c.key_pressed["forward"] = c.speed
                c.angryperf2 = False
                c.get_logger().info("Angry Movement 3!")
                return

            c.key_pressed["forward"] = 0.0

            # Sad
            if c.sadperforming or (c.notperformed and c.latest_emotion == "Sad"):
                c.key_pressed["forward"] = -c.speed
                c.get_logger().info("Sad Movement 1!")
                c.notperformed = False
                c.sadclean = True
                c.sadperforming = False
                return

            if c.sadclean:
                c.key_pressed["forward"] = 0.0
                c.get_logger().info("Sad Clean!")
                c.sadclean = False
                c.sadperf = True
                return

            if c.sadperf and c.count < 4:
                c.count += 1
                c.key_pressed["cw"] = c.speed
                c.get_logger().info("Sad Movement 2!")
                return
            elif c.sadperf:
                c.key_pressed["cw"] = 0.0
                c.count = 0
                c.sadperf = False

            # Surprised
            if c.surprisedperforming or (c.notperformed and c.latest_emotion == "Surprise"):
                c.key_pressed["th"] = c.speed
                c.get_logger().info("Surprised Movement 1!")
                c.notperformed = False
                c.surprisedclean = True
                c.surprisedperforming = False
                return

            if c.surprisedclean:
                c.key_pressed["th"] = 0.0
                c.get_logger().info("Surprised Clean!")
                c.surprisedclean = False
                if not c.surprisedperf2:
                    c.surprisedperf = True
                c.surprisedperf2 = False
                return

            if c.surprisedperf and c.count < 3:
                c.count += 1
                c.key_pressed["cw"] = c.speed * 2
                c.get_logger().info("Surprised Movement 2!")
                return
            elif c.surprisedperf:
                c.key_pressed["cw"] = 0.0
                c.count = 0
                c.surprisedperf = False
                c.surprisedperf2 = True
                return

            if c.surprisedperf2:
                c.key_pressed["th"] = -c.speed
                c.get_logger().info("Surprised Movement 3!")
                c.surprisedclean = True
                return

            # Fear
            if c.fearperforming or (c.notperformed and c.latest_emotion == "Fear"):
                c._land_pub.publish(Empty())
                c.get_logger().info("Fear Movement 1!")
                c.notperformed = False
                c.fearperforming = False
                return

            # Disgust
            if c.disgustperforming or (c.notperformed and c.latest_emotion == "Disgust"):
                c.key_pressed["forward"] = -c.speed * 2
                c.key_pressed["th"] = c.speed * 2
                c.get_logger().info("Disgust Movement 1!")
                c.notperformed = False
                c.disgustclean = True
                c.disgustperforming = False
                return

            if c.disgustclean:
                c.key_pressed["forward"] = 0.0
                c.key_pressed["th"] = 0.0
                c.get_logger().info("Disgust Clean!")
                c.disgustclean = False
                return
        else:
            c.get_logger().info("No emotion yet.")
