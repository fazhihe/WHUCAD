# WHUCAD

WHUCAD is the first full parametric and feature-based CAD dataset to support HCI in 3D learning, and therefore can support generating CAD models with real-world engineering features as advanced as that created by human engineers.

WHUCAD dataset and all code will be available only upon the acceptance of this paper "WHUCAD: A parametric and feature-based CAD dataset to support human-computer interaction for advanced 3D shape learning".

A gallery of WHUCAD dataset is shown above.

![image](https://github.com/fazhihe/WHUCAD/blob/main/A%20gallery%20of%20WHUCAD.png)

The vector format of WHUCAD is in the folder of ./data/vec . 
The b-rep format of WHUCAD is in the folder of ./data/brep . 

WHUCAD consists of two subsets. The first subset is derived from DeepCAD. We augment the data by randomly inserting sequences containing advanced features into DeepCAD data. The advantage of this approch is that it can obtain a rich amount of data with advanced features for learning and complex processing steps. We filter out unreasonable augmented data and check every model to ensure that WHUCAD has no redundant data. These models are in folders 0000-0099. The second subset comes from manual construction. We build 4,244 CAD models from the ABC dataset that have fruitful advanced features and semantic meaning but cannot be parsed into a command sequence by DeepCAD due to the lack of a selection mechanism. All models in the second subset of WHUCAD are macro recorded by CATIA and then parsed into vector sequence format. These models are in folders 0100-0103.
