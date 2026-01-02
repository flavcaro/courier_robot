#!/usr/bin/env python3
"""
AprilTag Image Generator

Generates PNG images for tag36h11 family (IDs 0-27) used in the courier robot simulation.
These images contain actual AprilTag encoded patterns that can be detected by AprilTag detectors.

Usage:
    python3 generate_apriltags.py
    
Output:
    Creates apriltag_images/ directory with 28 PNG files (tag_0.png to tag_27.png)
"""

import cv2
import numpy as np
import os


def generate_apriltag_images(output_dir='apriltag_images', tag_family='DICT_APRILTAG_36h11', 
                             num_tags=28, image_size=400, border_bits=1):
    """
    Generate AprilTag images for simulation.
    
    Args:
        output_dir: Directory to save PNG files
        tag_family: ArUco dictionary family (DICT_APRILTAG_36h11 for tag36h11)
        num_tags: Number of tags to generate (0 to num_tags-1)
        image_size: Size of output image in pixels (square)
        border_bits: White border around tag (1 = standard AprilTag border)
    """
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Get ArUco dictionary for tag36h11
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    print(f"Generating {num_tags} AprilTag images (tag36h11 family)...")
    print(f"Output directory: {output_dir}/")
    print(f"Image size: {image_size}x{image_size} pixels")
    print("-" * 50)
    
    for tag_id in range(num_tags):
        # Generate tag image
        tag_image = np.zeros((image_size, image_size), dtype=np.uint8)
        tag_image = cv2.aruco.generateImageMarker(aruco_dict, tag_id, image_size, tag_image, border_bits)
        
        # Save as PNG
        output_path = os.path.join(output_dir, f'tag_{tag_id}.png')
        cv2.imwrite(output_path, tag_image)
        
        print(f"✅ Generated: tag_{tag_id}.png")
    
    print("-" * 50)
    print(f"✅ Successfully generated {num_tags} AprilTag images!")
    print(f"📁 Location: {os.path.abspath(output_dir)}/")
    print()
    print("Next steps:")
    print("1. Images are ready to use in Gazebo simulation")
    print("2. world_spawner.py will load these PNG files as textures")
    print("3. AprilTag detector will recognize the encoded patterns")


if __name__ == '__main__':
    # Generate tags in the courier_nav package directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'apriltag_images')
    
    generate_apriltag_images(output_dir=output_dir, num_tags=28, image_size=512)
