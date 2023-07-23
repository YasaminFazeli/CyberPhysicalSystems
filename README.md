# 2023-group-08

[![pipeline](https://git.chalmers.se/courses/dit638/students/2023-group-08/badges/main/pipeline.svg)](https://git.chalmers.se/courses/dit638/students/2023-group-08/pipelines)

## Name
Dit639 project group08

## Introduction
This project is associated with dit639 (cyber physcial systems and systems of systems) course at unviersity of Gothenburg and is 
developed by group 8.

## Technologies and tools
- Cmake
- g++
- Docker
- Catch
- GitLab
- Git
- Ubuntu
- Docker-compose
- OpenDLV
- Vehicle View
- H264 Decoder
- OpenCV 



## Getting started

1. Create a folder and navigate to that folder.
```
mkdir group_08
cd group_08
```
2. Clone the repository.
```
git@git.chalmers.se:courses/dit638/students/2023-group-08.git

```
3. Go to the cpp-opencv file.
```
cd 2023-group-08/cpp-opencv
```
4. Open a new window in your VM terminal and navigate to the "recordings" folder.

5. Run the following commnad:

```
docker run --rm -i --init --net=host --name=opendlv-vehicle-view -v $PWD:/opt/vehicle-view/recordings -v /var/run/docker.sock:/var/run/docker.sock -p 8081:8081 chrberger/opendlv-vehicle-view:v0.0.64 
```
6. Open the following link in your browser and replace the letters with your VM IP address.
```
  http://A.B.C.D:8081

  ```
7. Open a new window in your VM terminal and run the following command to build a docker image.

```
docker build -f Dockerfile -t my-opencv-example .
```
8. Run the following commmands in your VM terminal.
```
xhost +
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp h264decoder:v0.0.5 --cid=253 --name=img

```
9. Start the recording in the openDLV platform.
10. Start your microservice with the following commmand.
```
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp my-opencv-example:latest --cid=253 --name=img --width=640 --height=480 --verbose 
```
Now you should be able to see the graphical user interfaces.


## Team workflow
### Code review checklist
When a merge request is made the person making the request shall assign another developer unaffialited with to review the merge request and check all the points below before approving or rejecting the request.

- [ ] Pull from the branch and make sure can be run
- [ ] Manually test the basic functionality of the code to make sure there are no obvious bugs (The remainder of the testing shall be handled by the CI)
- [ ] Visually inspect the added code including the test code
- Make sure the code is well documented and easy to understand 
- Look for any simple improvements to be made
- [ ] Make sure the code adheres to best practices to a reasonable extend
- [ ] Provide clear and concise feedback in the review section

### Adding Features
We follow the feature branch principle. So, for each new feature we first create an issue and put it in the corresponding milestone and if there is no corresponding milestone, we create one as well. We create new branch for each feature that is to be linked to the proper issue and milestone. Moreover, each feature shall has its own unit tests and must pass them and must be reviewed by another team member that is not assigned to that feature and also pass the CI pipeline before being merged to the master branch.

### Fix bugs
When a feature faces a bug, the lable of that feature goes from "feature" to "bug" and since the bugy feature is already merged to the master, a new bug branch specific to that issue/feature will be created including additional unit tests to ensure that the bug has been fixed properly. Afterwards, the same as above (Adding feature), applies to the rest of the flow.

## Commit messages
The commit message should start with (#) plus the serial number of the corresponding issue and then contain a descriptive summary of the changes in the imperative moood. If multiple developers have been contributing to the commit, they Co-Authoring shall be diclosed as the end in the following format:

```
#1 Implement this thing that does this thing.

Co-Authored-By: Amin Mahmoudifard <aminmah@student.chalmers.se>
```


***



## Description
The purpose of this project is to create a docker image, that can be run on a Raspberry Pi to compute the angle of the steering wheel.


## Usage
Using the data collected by the sensors of a model of an automated-vehicle to compute the angle of the steering wheel.

## Support
In case there are any problems you can contact the developers of this group.

## Roadmap
We have no plans for releasing this project as of now.

## Contributing
As of now we have no plans for contributions.

## Authors and acknowledgment
- [Yasamin Fazelidehkordi](https://git.chalmers.se/yasaminf)
- [Amin Mahmoudifard](https://git.chalmers.se/aminmah)
- Patrik Samcenko
- [Emrik Dunvald](https://git.chalmers.se/dunvald)
- [Labiba Karar Eshaba](https://git.chalmers.se/eshaba)

## License
[Licensed under the MIT license](https://git.chalmers.se/courses/dit638/students/2023-group-08/-/blob/Assignment6/LICENSE)

