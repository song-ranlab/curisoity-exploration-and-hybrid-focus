This folder contains the code created to generate the hybrid focus overlay on video images.  

The YOLO model was trained using the images found in the Stingray folder.  All images were hand labeled and training/testing data generated using a tool from: https://github.com/ManivannanMurugavel/Yolo-Annotation-Tool-New-
and following a tutorial at https://medium.com/@manivannan_data/how-to-train-yolov3-to-detect-custom-objects-ccbcafeb13d2

The ROST program was run seperatly.  Perplexity data was gained by running ROST sunshine on the video and saving the .json file generated.  The ROST program can be found at: https://gitlab.com/warplab/rost-cli

The darknet.py file used was obtained from https://github.com/pjreddie/darknet.

Also included are the programs used for generating perplexity figures and overlays used within the thesis (ppx_group_img_overlay.py and ppx_figure.py).
