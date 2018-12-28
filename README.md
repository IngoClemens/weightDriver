# weightDriver
Node for Autodesk Maya to create posed-based output values for driving relationships.

The weightDriver node offers two modes for creating driving relationships.

**Vector Angle** is a cone based pose reader to avoid rotational issues due to Euler rotations. While simple rotation dependencies can be mostly solved with expressions or set driven keys more elaborate setups are bound to fail when two or more axes are involved. A vector angle reader is usually a good and inexpensive solution in these cases.

For more complex setups the **RBF** mode (Radial Basis Function) delivers the best way of interpolating an arbitrary number of values based on any number of poses. This way it’s possible to basically drive anything from anything. A common application for a RBF solver is the blending of corrective shapes or blend shapes in general. Especially when it comes to problematic areas like the shoulder, wrist or hip area. The RBF mode can easily blend between different corrective shapes based on the orientation of the joint.

RBF mode overview on Vimeo: [RBF mode](https://vimeo.com/196583536)

The RBF mode of the weightDriver is also supported through the more advanced [RBF Manager](https://www.youtube.com/watch?v=VyWCaE-YOwk) of the [mGear rigging framework](http://www.mgear-framework.com).

The weightDriver node is also part of the [SHAPES](http://www.braverabbit.com/shapes/) plug-in, our advanced blend shape editor for Maya. If you have SHAPES installed you already have the weightDriver node available.
The difference between the standalone RBF mode and the RBF mode used in SHAPES is that the former has single-value based poses whereas the latter uses matrix-based poses.

**weightDriver is under the terms of the MIT License**

## Installation

For ease of use all files are combined as a module. This allows for an easy installation and keeps all necessary files in one location.

**_Important:_**

**_If after the installation the plug-in doesn't show up in the plug-in manager it's possible that the downloaded files from github have faulty user permissions. In this case try to create the modules folder manually and check the permissions or download the file from [braverabbit](http://www.braverabbit.com/weightdriver/)._**

Copy the module folder from the repository to your Maya preferences. The module is version independent which means it can be installed in the preferences root folder.

The Maya preferences root directory is located at:

    Windows: C:\Users\USERNAME\Documents\maya
    macOS: /Users/USERNAME/Library/Preferences/Autodesk/maya
    Linux: /home/USERNAME/maya

A default Maya installation doesn't have a modules folder at this specified path. You can directly use the folder from the repository. If the modules folder already exists copy the contents of the repository's modules folder to the one in your preferences.

Inside the modules folder, rename the module template file, which matches your operating system, by removing the current extension. The file should be named weightDriver.mod.

Edit the file in a text editor and replace USERNAME in the paths with your user name. Save the file.

Restart Maya. Load the weightDriver plug-in from the plug-in manager.

## Usage

When properly installed the weightDriver plug-in should appear in the plug-in manager.

The weightDriver node can either be created manually or managed through the accompanying editor. Manual creation is used in case of utilizing the vector angle mode. The editor handles the pose related data when in RBF mode.

To start using the node use one of the following commands:

`createNode weightDriver` to create the node manually.

`weightDriverEditRBF` automatically loads the plug-in and opens the Weight Driver Editor.

The weightDriver node can also be used in RBF mode through the [RBF manager](https://www.youtube.com/watch?v=VyWCaE-YOwk) of [mGear](http://www.mgear-framework.com)

## Usage - Vector Angle

In order to make the node work correctly you need to establish two connections.

The first tells the node the reference point for the vector calculation and is usually the transform node of the weightDriver shape node. The second connection comes from the node which serves as the target point for the vector, such as a locator, joint or other node.

- connect the weightDriver transform node worldMatrix to the weightDriverShape readerMatrix (depending on the editor you use you might have to display hidden attributes in oder to see the reader and driver inputs)
- create a locator and connect the locator transform node worldMatrix attribute to the weightDriverShape driverMatrix
- with both connections established the weightDriver node now displays the driver name and weight, which is the outWeight attribute you can now connect to any attribute you want to drive, for example a blend shape channel.

## Usage - RBF

In most cases using the designated editor to create or edit a RBF setup is sufficient and the preferred way.
However, in some cases it might be necessary to manually setup the weightDriver in RBF mode. In general, the following should be considered:

**Rest Poses**
Rest poses, if needed, should come first in the post list.

The reason for this is that Maya stores sparse arrays for it’s attributes, which means that if an array attribute contains a zero, it won’t get stored in the file. Rest poses often have many zeros and thus aren’t stored correctly. This can cause false RBF behavior when loading a scene file. If a pose, with zero values for the input and pose values is placed at the end of the pose list the weight driver is unable to tell how many poses there are when loading a scene file.

**Successive Pose IDs**
When connecting attributes manually without the Weight Driver Editor all connections should be made in successive order, starting at index 0. Indices should not be skipped. When using the editor, this is not an issue, because even if poses get deleted the resulting setup will always have the poses in successive order.

Again, this has to do with Maya’s sparse arrays. Manually skipping one or more indices can lead to a false RBF behavior.

### Latest version: 3.6.0 (2018-12-28)


## Changelog

**3.6.0 (2018-12-28)**

    - Initial open source release.
    - New kernel attribute which allows to switch between linear and gaussian RBF interpolation. In previous versions only gaussian interpolation existed. Linear interpolation creates slightly different results and may be more appropriate in certain setups but should be tested. Gaussian interpolation is the default.
