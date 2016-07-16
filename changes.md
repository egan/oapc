Changes Since Provided Version
==============================
Cosmetic:
---------
* Cleaned up whitespace and indentation.
* Renamed arithmetic routines for consistency.
* Renamed data names for consistency.
* Changed case of names, mnemonics for consistency.
* Changed hex constant notation for consistency.
* Organized data definitions.
* Removed inaccurate ORG comments.

Documentation:
--------------
* Added code section headers.
* Improved and corrected all documentation headers for routines.
	- Name, description.
	- Inputs, outputs.
	- Side effects.
* Improved and corrected all inline documentation of algorithms.

Code:
-----
* Added timing functionality, always on.
* Added defensive switch to register bank 3 to all arithmetic routines.
* Corrected precision mismatches in `INTPAndFXCheck`.
* Removed nonfunctional `FRound` routine.
