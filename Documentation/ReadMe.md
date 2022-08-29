# What should be in the documentation folder


## Signoffs

An important part of engineering is analysis. Designs must be analyzed to show that the subsystem is **likely** to function (sometimes things still don't work). This reduces the overall risk that a client or firm will have wasted money on a solution that was destined to fail. For information about what must be included in each signof file, refer to the readme file in the signoff directory. 

A signoff must be completed and approved for each subsystem before the components for the subsystem may be ordered. Each of the signoff documents should be an appropriately named markdown file in the signoff directory. 


## Final set of prints (final design artifacts)

All schematics and artifacts should be in an appropriate file type. 

### 3D Models

Required format: 
- .stl

3D models should be stored in the proper file type to facilitate editing, viewing, and printing. The best filetype for this is .stl, so all 3d models should be pushed to the repo as stl files. Stl files can be converted to solidworks models or autocad 3d models as necessary, can be 3d printed easily, and are 3d viewable natively in github. File (Documentation/3D Models)

Every constructed system of the project must have a complete (buildable) 3d model schematic.


### Wiring Schematics

Required software: 
- autocad or autocad electrical

For wiring schematics, use autocad or autocad electrical. All the source files must be included in the github repo along with pdf versions of all schematics. File (Documentation/Electrical/Schematics/Sources)

Every electrical system must have a complete (buildable) pdf circuit schematic. File (Documentation/Electrical/Schematics)

Autocad is freely available to students with versions for mac, windows, and linux.

#### PCB files

Required Software:
- kicad 

KiCad should be used for all PCB design. All source files must be deposited in the repo. File (Documentation/Electrical/PCB/Sources)

Along with the source files, a complete set of gerber files for each PCB must also be included in the repo. These should be included in a folder called gerber files. File (Documentation/Electrical/PCB)

kiCad is a free and open source software.


## Final BOM

The final version of the BOM should be uploaded as a pdf output of the excel BOM file. The excel BOM file should also be included.


## Datasheet 

This must give all the information necessary for other systems to interface with your project in the intended manner. This must include the power expectations, interfacing, and compliance.
	
This must also include the results from all experimentation and the interpretation of the results.
  
  
  
  
## The project poster

Include the project poster


## Detailed photos of the project

Include a folder of a set of detailed photos of the project. File (Documentation/Images)


## Any additional documents that are appropriate to include

If there are additional items that are important to understanding and working on or with the project, include them as well. 
