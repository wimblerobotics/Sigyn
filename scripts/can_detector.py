# from inference import InferencePipeline
# import cv2

# def my_sink(result, video_frame):
#     if result.get("output_image"): # Display an image from the workflow response
#         cv2.imshow("Workflow Image", result["output_image"].numpy_image)
#         cv2.waitKey(1)
#     # Do something with the predictions of each frame
#     print(result)


# # 2. Initialize a pipeline object
# pipeline = InferencePipeline.init_with_workflow(
#     api_key="DorJsZuWhtXeASebA0nm",
#     workspace_name="s-irbpz",
#     workflow_id="find-cokezerocans",
#     video_reference="/home/ros/sigyn_ws/training_images/20250601_093809.jpg", # Test image
#     max_fps=30,
#     on_prediction=my_sink
# )

# # 3. Start the pipeline and wait for it to finish
# pipeline.start()
# pipeline.join()
# 1. Import the InferencePipeline library
from inference import InferencePipeline
import cv2

def my_sink(result, video_frame):
    # Print detection results
    print(result)
    
    # Save annotated image if available
    if result.get("visualization"):
        img = result["visualization"].numpy_image
        output_path = "/home/ros/sigyn_ws/src/Sigyn/scripts/detection_result.jpg"
        cv2.imwrite(output_path, img)
        print(f"Saved annotated image to: {output_path}")


# 2. Initialize a pipeline object
pipeline = InferencePipeline.init_with_workflow(
    api_key="DorJsZuWhtXeASebA0nm",
    workspace_name="s-irbpz",
    workflow_id="find-cokezerocans",
    video_reference="/home/ros/sigyn_ws/training_images/CokeZeroCan/20260120_124033.jpg", # Test image
    # video_reference=0, # Path to video, device id (int, usually 0 for built in webcams), or RTSP stream url
    max_fps=30,
    on_prediction=my_sink
)

# 3. Start the pipeline and wait for it to finish
pipeline.start()
pipeline.join()
