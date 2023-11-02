**4.0.1 (2023-11-02)**
* Fixed that poses don't produce a full output weight with the RBF mode set to Matrix.
* Fixed a vector angle weight output error.

**4.0.0 (2023-06-22)**
* RBF algorithm update. This breaks compatibility with previous versions of the weightDriver node.
* Minimum supported Maya version is 2020.
* Added new kernel types and radius options.
* Removed the bias value.
* Improved error message in case of a calculation error.

**3.6.2 (2023-06-12)**
* Added support for Maya 2024.
* Removed an obsolete part for Maya 2024.

**3.6.1 (2022-06-01)**
* Added support for Maya 2023.
* Fixed the loss of stored data in generic mode after the Active check box has been toggled to temporarily bypass the solver.

**3.6.0 (2021-03-29) - Update**
* Added support for Maya 2022.

**3.6.0 (2020-03-07) - Update**
* Added support for Maya 2020.

**3.6.0 (2018-12-28)**
* Initial open source release.
* New kernel attribute which allows to switch between linear and gaussian RBF interpolation. In previous versions only gaussian interpolation existed. Linear interpolation creates slightly different results and may be more appropriate in certain setups but should be tested. Gaussian interpolation is the default.
