"""One material table used by every backend.

visual   : what each renderer can use. PyBullet only uses rgba (+ a specular colour
           derived from `metallic`); Isaac uses rgba/metallic/roughness via
           UsdPreviewSurface, or the NVIDIA MDL material named in `mdl` if enabled.
physical : density (mass = mesh volume x density), friction, restitution.
optical  : properties the D435 camera model will read (filled in once we decide
           which effects to model; deliberately empty for now).

Values are typical handbook numbers, not measurements:
- aluminium density 2700 kg/m^3 (6063-T5 extrusion alloy: 2690)
- die-cast zinc (Zamak) 6600 kg/m^3 -> bare bracket 15.6 g (item's ~23 g likely includes screws and nuts)
- dry friction aluminium vs. painted / laminated table: ~0.3-0.5 -> 0.4
"""

MATERIALS = {
    "aluminium_anodized": {   # clear-anodized extrusions: satin, fairly diffuse
        "visual": {"rgba": (0.78, 0.79, 0.81, 1.0), "metallic": 1.0, "roughness": 0.45,
                   "mdl": "Materials/Base/Metals/Aluminum_Anodized.mdl"},
        "physical": {"density": 2700.0, "friction": 0.4, "restitution": 0.05},
        "optical": {},
    },
    "aluminium_bare": {       # machined / bare aluminium: shiny, strongly specular
        "visual": {"rgba": (0.91, 0.92, 0.92, 1.0), "metallic": 1.0, "roughness": 0.15,
                   "mdl": "Materials/Base/Metals/Aluminum_Polished.mdl"},
        "physical": {"density": 2700.0, "friction": 0.4, "restitution": 0.05},
        "optical": {},
    },
    "zinc_diecast_ral9006": { # item angle bracket: die-cast zinc, powder-coated RAL 9006 (white aluminium,
                              # a metallic-effect paint): satin, much more diffuse than bare metal
        "visual": {"rgba": (0.65, 0.66, 0.66, 1.0), "metallic": 0.5, "roughness": 0.5, "mdl": None},
        "physical": {"density": 6600.0, "friction": 0.4, "restitution": 0.05},   # Zamak die-cast alloy
        "optical": {},
    },
    "table_grey": {           # matte painted / laminated work surface
        "visual": {"rgba": (0.35, 0.35, 0.37, 1.0), "metallic": 0.0, "roughness": 0.8, "mdl": None},
        "physical": {"density": None, "friction": 0.5, "restitution": 0.1},
        "optical": {},
    },
}
