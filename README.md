<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->

<a name="readme-top"></a>

<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://www.roboworks.net/">
    <img src="images/roboworks-logo.png" alt="Logo" width="240" height="240">
  </a>

  <h3 align="center">Roboworks</h3>

  <p align="center">
    This project contains all the packages neccessary to run Roboworks products
    <br />
    <a href="https://github.com/Roboworks-Global/transbot"><strong>Explore the docs »</strong></a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

</br>

<!-- ABOUT THE PROJECT -->

## About The Project

<p align="center">
  <a href="https://www.roboworks.net/store-homepage">
  <img src="images/rosbot-plus.png" alt="Logo" width="240" height="240">
  </a>
</p>

Mecabot X is an ideal Autonomous Mobile Robot (AMR) platform for indoor service robot applications with full metallic enclosure.

Mecabot X is equipped with builtin ROS Controller, LiDAR, Depth Camera, STM32 Motor/Power/IMU and metal chassis with 4 Mecanum wheels and Independent Suspension Systems.

Mecabot X saves time and money for rapid prototyping or product development for indoor service robot projects. With its metallic enclosure, it is a ready-to-ship product for your target market. This is a serious professional AMR platform to be integrated into service robots especially for indoor environment such as factories, warehouses, hospitals, public transports, office buildings, hotels and restaurants.

It can travel up to 1.39 km/h. Its mecanum wheels can allow Mecabot X travel in omnidirectional directions. Mecabot X can navigate in smaller indoor environment with greater flexibility. The max payload is 60 kg.

Mecabot X comes with revolutionary Power Management Units including Power Mag and Auto Charge. Power Mag is a magnetic LFP battery allows quick and easy battery swaps. Mecabot XS takes it further, integrating Auto Charge, the automatic charging solution.

Both Mecabot X and Mecabot XS come pre-loaded with MiROS, a cloud-based ROS DevKit with a user-friendly graphical interface. They also include free tutorials, supported by a global ROS community.

Mecabot X is one of the most agile AMR platforms that can carry out big jobs at low cost.

### Technical Specifications

- ROS Controller - Orin Nano/Orin NX,
- Power Mag - Magnetic LFP Battery and Battery Charger
- Orbbec Depth Camera
- LS LiDAR - M10P (30m Detection Range)
- Auto Charge (auto charging station) for Mecabot XS models
- Flight case

### Dimensions

- Height: 203mm
- Width: 581mm
- Length: 630mm
- Weight: 20.5kg

### Power Supply

- 24V 6100mAh battery
- 3A DC Charger
- 4 hours battery life (without loading)
- 3 hours battery life (with 3kg loading)

### Motor Specifications

- MD60 100W DC Brushed Motor
- 1:18 Reduction Ratio
- Wheel diameter: 152mm
- Max. Payload: 60kg
- Max. Speed: 1.39 m/s

### I/O Interfaces

- CAN
- Serial Ports
- USB
- HDMI

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->

## Getting Started

### Prerequisites

The packages in this repository is built on the ROS2 framework. The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. Please refer to the following URL and install ROS2 humble and tools.

- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html
- https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Debian-Package.html

Ceratin packages also require specific dependencies, please refer to the individual READMEs for each package.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

1. Set up SSH keys on Github
2. Clone the repo into your workspace src directory
   ```sh
   mkdir ~/ros2_ws ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Roboworks-Global/transbot -b <robot_model>
   ```
3. Install dependencies
   ```sh
   cd ~/ros2_ws
   rosdep install --ignore-src --from-path src/ -r -y
   ```
4. Build the packages
   ```sh
   colcon build --symlink-install
   ```
5. Source the install

   ```sh
   . install/setup.bash
   ```

6. Your workspace should look like the following

```
~/ros2_ws/
└─ build/
└─ log/
└─ install/
    └─ setup.bash
    └─ ...
└─ src/
    └─ transbot/
        └─ core/
        └─ navigation/
        └─ ...
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Setup

1. Run the following script to set up the serial devices:

   ```sh
   sudo chmod +x ./src/transbot/turn_on_wheeltec_robot/wheeltec_udev.sh
   sudo ./src/transbot/turn_on_wheeltec_robot/wheeltec_udev.sh
   ```

2. Follow the README in the ros2_astra_camera package to set up the camera

TODO

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->

## Usage

To launch a package use the following command:

```sh
  ros2 launch <package name> <launch file>
```

Example:

```sh
  ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

## Debugging

Debugging tools are available in the ROS2 framework.

- rqt
- rviz

TODO

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->

## Contributing

TODO

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->

## License

Copyright 2023 Roboworks Pty Ltd

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->

## Contact

TODO

Your Name - [@your_twitter](https://twitter.com/your_username) - email@example.com

Project Link: [https://github.com/your_username/repo_name](https://github.com/your_username/repo_name)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->

## Acknowledgments

Use this space to list resources you find helpful and would like to give credit to. I've included a few of my favorites to kick things off!

- [Choose an Open Source License](https://choosealicense.com)
- [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
- [Malven's Flexbox Cheatsheet](https://flexbox.malven.co/)
- [Malven's Grid Cheatsheet](https://grid.malven.co/)
- [Img Shields](https://shields.io)
- [GitHub Pages](https://pages.github.com)
- [Font Awesome](https://fontawesome.com)
- [React Icons](https://react-icons.github.io/react-icons/search)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[ros]: https://img.shields.io/ros/v/galactic/rclcpp
[ros-url]: https://docs.ros.org/en/galactic/index.html
