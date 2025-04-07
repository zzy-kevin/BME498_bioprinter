#This project is based on Meta's SAM2 model https://github.com/facebookresearch/sam2#model-description

from PIL import Image
import numpy as np
import os
# if using Apple MPS, fall back to CPU for unsupported ops

import torch
import matplotlib.pyplot as plt

from ui import * 

#wound_img = Image.open('./img/wound.png')


def segmentation_main(wound_img):

    x,y=select_point(wound_img)
    wound_img = np.array(Image.open(wound_img).convert("RGB"))

    os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"



    # select the device for computation
    if torch.cuda.is_available():
        device = torch.device("cuda")
    elif torch.backends.mps.is_available():
        device = torch.device("mps")
    else:
        device = torch.device("cpu")
    print(f"using device: {device}")

    if device.type == "cuda":
        # use bfloat16 for the entire notebook
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
    elif device.type == "mps":
        print(
            "\nSupport for MPS devices is preliminary. SAM 2 is trained with CUDA and might "
            "give numerically different outputs and sometimes degraded performance on MPS. "
            "See e.g. https://github.com/pytorch/pytorch/issues/84936 for a discussion."
        )


    np.random.seed(3)

    def show_mask(mask, ax, random_color=False, borders = True):
        if random_color:
            color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
        else:
            color = np.array([30/255, 144/255, 255/255, 0.6])
        h, w = mask.shape[-2:]
        mask = mask.astype(np.uint8)
        mask_image =  mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
        if borders:
            import cv2
            contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
            # Try to smooth contours
            contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
            mask_image = cv2.drawContours(mask_image, contours, -1, (1, 1, 1, 0.5), thickness=2) 
        ax.imshow(mask_image)

    def show_points(coords, labels, ax, marker_size=375):
        pos_points = coords[labels==1]
        neg_points = coords[labels==0]
        ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
        ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)   

    def show_box(box, ax):
        x0, y0 = box[0], box[1]
        w, h = box[2] - box[0], box[3] - box[1]
        ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0, 0, 0, 0), lw=2))    

    def show_masks(image, masks, scores, point_coords=None, box_coords=None, input_labels=None, borders=True, save=False):
        for i, (mask, score) in enumerate(zip(masks, scores)):
            plt.figure(figsize=(10, 10))
            plt.imshow(image)
            show_mask(mask, plt.gca(), borders=borders)
            if point_coords is not None:
                assert input_labels is not None
                show_points(point_coords, input_labels, plt.gca())
            if box_coords is not None:
                # boxes
                show_box(box_coords, plt.gca())
            if len(scores) > 1:
                plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
            plt.axis('off')
            #save the image in the directory
            if save:
                filename = 'segmented_mask.png'
                plt.savefig('segmented_mask.png')
            plt.show()
            break    #shows only the first mask, usually the first ohe is the most accurate one
        return filename
        




    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor

    sam2_checkpoint = "./checkpoints/sam2.1_hiera_large.pt"
    model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"

    sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=device)

    predictor = SAM2ImagePredictor(sam2_model)
    predictor.set_image(wound_img)

    input_point = np.array([[x,y]])  #TODO: User Input
    input_label = np.array([1])    #TODO: User Input

    masks, scores, logits = predictor.predict(
        point_coords=input_point,
        point_labels=input_label,
        multimask_output=True,
    )
    sorted_ind = np.argsort(scores)[::-1]
    masks = masks[sorted_ind]
    scores = scores[sorted_ind]
    logits = logits[sorted_ind]

    show_masks(wound_img, masks, scores, point_coords=input_point, input_labels=input_label, borders=True, save=True)

    #show_masks(wound_img, masks, scores, point_coords=input_point, input_labels=input_label, borders=True, save=True)

    return masks
    #show_mask(masks, plt.gca(), random_color=False, borders = True)

