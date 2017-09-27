SET_MODULE_TYPE_LOADABLE_MODULE
// modules that the new module is based on. expand as required.
ADD_DEPENDENCY("x4basics")
ADD_DEPENDENCY("x4tools")
ADD_DEPENDENCY("x4modules")

// add one or more authors (repeat command for multiple authors)

// add the institution holding the copyright

// BSD is the default license. Choose carefully for your new module.
ADD_LICENSE("BSD")

// an icon for this module
ADD_ICON(":/images_projectsEG2014/images/projectsEG2014.png")

// Add classes here: format is "class", "baseClass". class.h is included
// ADD_CLASS("SomeNewClass", "Persistent")

// Alternative: specify file to include for this declaration
// ADD_CLASS_INC("SomeNewClass", "Persistent", "AHeaderWithManyClasses.h")
ADD_CLASS("AttachmentBuildingBlockUpward", "AttachedData")
ADD_CLASS("PCIBuildingBlockEG2014", "PCInteractionTool")