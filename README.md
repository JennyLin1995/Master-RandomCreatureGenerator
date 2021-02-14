# Random Creature Generator

Master Project: This project was created as part of a masters thesis about "Procedural Generation of Creatures" 
by Jennifer Lindner, s0549868, HTW Berlin (University of Applied Sciences), 2020-2021  

## How to use

### Prerequisites

1) Download and install Blender 2.8x from [Blender.org](https://www.blender.org/) 
2) Start a new blender project and enable the following modules under Edit > Preferences > Add-ons (search for "extra"):
- Add Curve: Extra Objects
- Add Mesh: Extra Objects

### Using the Script

Either download the project by cloning the Repository or simply download the RandomCreatureGenerator.py
In Blender, use the "Scripting" Workspace. In the editor open the downloaded file (Text > Open), or create a new script and paste the code there.
Using the Play Button or Text > Run Script will run the generator and create a random new creature

## Changing values

Many variables and parameters used can be changed to slightly adjust the outcome of the generated creature. 
Noticably changing the "Seed" value to anything not empty will cause the generator to return the same creature even upon executing the code again.
You can turn of smoothing to view the original mesh that is generated by changing the values of the global parameters. (E.G. line 38+) 
```
SEED = ""    
MERGE_OBJECTS = True
SMOOTH_OBJECTS = True
```

## Authors

* **Jennifer Lindner** - *Initial work* - [JennyLin1995](https://github.com/JennyLin1995)

## License

This project is licensed under the MIT License - see the [MIT License](MIT-License.txt) file for details

## Acknowledgments

Thanks to [the team behind the spaceship generator](https://github.com/a1studmuffin/SpaceshipGenerator) for their interesting approach and some helpfull functions 

