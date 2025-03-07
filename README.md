# Capstone: Insulation Monitoring

## Overview
This is a ROS workspace that monitors the insulation level of horizontal cavities during insulation blowing and publishes commands for adjusting the aim. It  takes in RGBD and lidar sensor point cloud data and filters out the airborne insulation and calculates the centroid, variance, and bounding box of the region where insulation is being blown. It then creates an elevation map of accumulated insulation. It publishes a command for how much the centroid of the insulation region of interest (ROI) should move in the x,y plane.

## Installation
This software requires the installation of the Robotic Operating System (ROS) as well as Grid Map and Eigen, which are Elevation Mapping dependencies.

### Building
Clone respository and compile.
```bash
cd catkin_ws/src
git clone https://github.com/clee009/insul_monit_ws.git
cd ../
catkin build
```

## Usage
You will need to run two separate nodes, one for filtering/insulation ROI/aim adjustment and the other for elevation mapping
```bash
roslaunch insul_monit insul_monit.launch
roslaunch elevation_mapping elevation_mapping.launch
```

## Nodes
Node: 

## Dependencies
- Python 3.8+
- OpenCV
- NumPy
- ROS *(if applicable)*

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -m "Added new feature"`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a Pull Request.

## License
This project is licensed under the [MIT License](LICENSE).

## Contact
For questions or support, contact me at:  
Email: `your-email@example.com`  
Website: [your-website.com](https://your-website.com)  
GitHub: [your-username](https://github.com/your-username)

## Acknowledgements *(optional)*
- Thanks to [Name](https://link) for their contributions.
- Inspired by [Project](https://link).

## Additional Resources *(optional)*
- [Project Wiki](https://link)
- [API Documentation](https://link)

## Badges *(optional)*
You can add GitHub workflow status, version, or other badges like this:
```
![GitHub Workflow Status](https://img.shields.io/github/workflow/status/your-username/your-repo/CI)
![License](https://img.shields.io/github/license/your-username/your-repo)
![Contributors](https://img.shields.io/github/contributors/your-username/your-repo)
```

