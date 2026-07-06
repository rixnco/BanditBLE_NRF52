"""
Générateur de fichier Fritzing (.fzz) pour le montage BanditBLE GSF650.

Circuit :
  - Seeed XIAO nRF52840 (MCU principal)
  - TLP291 (DIP-8, opto-isolateur ABS)
  - R1 100Ω  : limitation courant LED TLP291
  - R2 10kΩ  : pull-up 3.3V sur Collector TLP291
  - R3 10kΩ  : pull-down GPIO P0.05
  - C1 100nF : découplage côté photo TLP291
  - C2 10µF  : découplage côté LED (entrée ABS)
  - D1 1N4148: protection polarité inverse
  - Connecteurs : ABS in, VRS crankshaft, Gear ADC
"""

import zipfile
import uuid

# ──────────────────────────────────────────────────────────────────────────────
# Helpers positionnement breadboard (demi-breadboard 30 colonnes × 10 rangées)
# Fritzing : espacement trous ≈ 9 px, origine breadboard = (10, 10)
# ──────────────────────────────────────────────────────────────────────────────
BOARD_X   = 10      # px origine breadboard
BOARD_Y   = 10      # px origine breadboard
HOLE_STEP = 18      # px entre deux trous (0.1 inch à 180 dpi)
COL_OFFSET = 28     # px jusqu'au premier trou colonne 1
ROW_OFFSET = 68     # px jusqu'au premier trou rangée a

def bpos(col: int, row_char: str) -> tuple[float, float]:
    """Coordonnées pixel d'un trou breadboard.
    col : 1..30
    row_char : 'a'..'e' (section haute) ou 'f'..'j' (section basse)
    """
    row_map = {'a': 0, 'b': 1, 'c': 2, 'd': 3, 'e': 4,
               'f': 5, 'g': 6, 'h': 7, 'i': 8, 'j': 9}
    x = BOARD_X + COL_OFFSET + (col - 1) * HOLE_STEP
    # section f-j : saut de 12px pour l'espace central
    gap = 12 if row_map[row_char] >= 5 else 0
    y = BOARD_Y + ROW_OFFSET + row_map[row_char] * HOLE_STEP + gap
    return round(x, 1), round(y, 1)


# ──────────────────────────────────────────────────────────────────────────────
# Instance helpers
# ──────────────────────────────────────────────────────────────────────────────

def inst_resistor(idx: int, title: str, note: str, ohm: str, x: float, y: float, vertical: bool = True) -> str:
    transform = "0,1,-1,0,0,0" if vertical else "1,0,0,1,0,0"
    return f"""
    <instance moduleIdRef="ResistorModuleID" modelIndex="{idx}" path="core/resistor.fzp">
      <title>{title}</title>
      <text>{note}</text>
      <views>
        <breadboardView>
          <geometry x="{x}" y="{y}" z="2.0" transform="{transform}"/>
        </breadboardView>
        <schematicView>
          <geometry x="{x+400}" y="{y}" z="2.0"/>
        </schematicView>
      </views>
      <properties>
        <property name="resistance" showInLabel="yes">{ohm}</property>
        <property name="pin spacing">400mil</property>
        <property name="package">THT</property>
        <property name="tolerance">5%</property>
        <property name="layers">1</property>
      </properties>
    </instance>"""


def inst_cap_ceramic(idx: int, title: str, note: str, cap: str, x: float, y: float) -> str:
    return f"""
    <instance moduleIdRef="CerDiscCapacitorModuleID" modelIndex="{idx}" path="core/ceramic_capacitor.fzp">
      <title>{title}</title>
      <text>{note}</text>
      <views>
        <breadboardView>
          <geometry x="{x}" y="{y}" z="2.0" transform="0,1,-1,0,0,0"/>
        </breadboardView>
        <schematicView>
          <geometry x="{x+400}" y="{y}" z="2.0"/>
        </schematicView>
      </views>
      <properties>
        <property name="capacitance" showInLabel="yes">{cap}</property>
        <property name="package">THT</property>
        <property name="voltage">50V</property>
      </properties>
    </instance>"""


def inst_cap_elec(idx: int, title: str, note: str, cap: str, volt: str, x: float, y: float) -> str:
    return f"""
    <instance moduleIdRef="ElectrolyticCapacitorModuleID" modelIndex="{idx}" path="core/electrolytic_capacitor.fzp">
      <title>{title}</title>
      <text>{note}</text>
      <views>
        <breadboardView>
          <geometry x="{x}" y="{y}" z="2.0" transform="0,1,-1,0,0,0"/>
        </breadboardView>
        <schematicView>
          <geometry x="{x+400}" y="{y}" z="2.0"/>
        </schematicView>
      </views>
      <properties>
        <property name="capacitance" showInLabel="yes">{cap}</property>
        <property name="voltage" showInLabel="yes">{volt}</property>
        <property name="package">THT</property>
      </properties>
    </instance>"""


def inst_diode(idx: int, title: str, note: str, diode_type: str, x: float, y: float) -> str:
    return f"""
    <instance moduleIdRef="DiodeModuleID" modelIndex="{idx}" path="core/diode.fzp">
      <title>{title}</title>
      <text>{note}</text>
      <views>
        <breadboardView>
          <geometry x="{x}" y="{y}" z="2.0" transform="0,1,-1,0,0,0"/>
        </breadboardView>
        <schematicView>
          <geometry x="{x+400}" y="{y}" z="2.0"/>
        </schematicView>
      </views>
      <properties>
        <property name="type" showInLabel="yes">{diode_type}</property>
        <property name="package">THT</property>
      </properties>
    </instance>"""


def inst_label(idx: int, title: str, text: str, x: float, y: float) -> str:
    """Note textuelle (part Label de Fritzing)."""
    return f"""
    <instance moduleIdRef="NoteModuleID" modelIndex="{idx}" path="core/note.fzp">
      <title>{title}</title>
      <text>{text}</text>
      <views>
        <breadboardView>
          <geometry x="{x}" y="{y}" z="99.0" transform="1,0,0,1,0,0"/>
        </breadboardView>
      </views>
    </instance>"""


def inst_wire(idx: int, x1: float, y1: float, x2: float, y2: float,
              color: str = "#efef00") -> str:
    """Fil de connexion visible sur breadboard view."""
    return f"""
    <instance moduleIdRef="WireModuleID" modelIndex="{idx}" path="core/wire.fzp">
      <title>Wire</title>
      <views>
        <breadboardView>
          <geometry x="{x1}" y="{y1}" z="9.0" transform="1,0,0,1,0,0"/>
          <wireExtras mils="0" x1="0" y1="0" x2="{x2-x1}" y2="{y2-y1}" color="{color}" thickness="1.7" opacity="1"/>
        </breadboardView>
      </views>
      <connectors>
        <connector connectorId="connector0" layer="breadboard">
          <geometry x="0" y="0"/>
          <connects/>
        </connector>
        <connector connectorId="connector1" layer="breadboard">
          <geometry x="{x2-x1}" y="{y2-y1}"/>
          <connects/>
        </connector>
      </connectors>
    </instance>"""


# ──────────────────────────────────────────────────────────────────────────────
# Coordonnées des composants sur la breadboard
# Layout (colonnes 1..30, rangées a..j) :
#
#   Col  1- 4 : alimentation (3V3, GND rails)
#   Col  5-16 : XIAO nRF52840 (DIP footprint simulé)
#   Col 18-25 : TLP291 (DIP-8, cols 18-21 en section a-d)
#   Col 18    : R1 100Ω  (ABS in → TLP291 pin1)
#   Col 22    : R2 10kΩ  (pull-up, col 22 section f-g)
#   Col 24    : C1 100nF (col 24 section f-g)
#   Col 16    : C2 10µF  (col 16 section h-i)
#   Col 26    : R3 10kΩ  (pull-down, col 26 section f-g)
#   Col 28    : D1 1N4148 (col 28 section f-g)
# ──────────────────────────────────────────────────────────────────────────────

# Positions clés
xiao_x, xiao_y = bpos(5, 'a')   # coin XIAO

tlp_pin1_x,  tlp_pin1_y  = bpos(19, 'b')   # Anode LED  (pin1)
tlp_pin2_x,  tlp_pin2_y  = bpos(19, 'c')   # Cathode LED (pin2)  → GND
tlp_pin4_x,  tlp_pin4_y  = bpos(19, 'd')   # GND photo (pin4)   → GND
tlp_pin6_x,  tlp_pin6_y  = bpos(22, 'd')   # Emitter (pin6)
tlp_pin7_x,  tlp_pin7_y  = bpos(22, 'c')   # Collector (pin7)

r1_x, r1_y   = bpos(17, 'b')   # R1 100Ω
r2_x, r2_y   = bpos(22, 'a')   # R2 10kΩ pull-up
r3_x, r3_y   = bpos(26, 'd')   # R3 10kΩ pull-down
c1_x, c1_y   = bpos(24, 'c')   # C1 100nF
c2_x, c2_y   = bpos(16, 'b')   # C2 10µF
d1_x, d1_y   = bpos(28, 'c')   # D1 1N4148


# ──────────────────────────────────────────────────────────────────────────────
# Construction du sketch XML
# ──────────────────────────────────────────────────────────────────────────────

instances_xml = ""

# 1. Breadboard (demi-taille)
instances_xml += f"""
    <instance moduleIdRef="BreadboardHalfModuleID" modelIndex="1" path="core/breadboard_half.fzp">
      <title>BB</title>
      <text>Demi-breadboard 400 trous</text>
      <views>
        <breadboardView>
          <geometry x="{BOARD_X}" y="{BOARD_Y}" z="0.5" transform="1,0,0,1,0,0"/>
        </breadboardView>
        <schematicView>
          <geometry x="0" y="0" z="0.5" visible="false"/>
        </schematicView>
        <pcbView>
          <geometry x="0" y="0" z="0.5" visible="false"/>
        </pcbView>
      </views>
    </instance>"""

# 2. Seeed XIAO nRF52840  → Arduino Nano comme base (même brochage DIP)
instances_xml += f"""
    <instance moduleIdRef="Arduino NanoModuleID" modelIndex="2" path="core/arduino_nano.fzp">
      <title>U0 XIAO-nRF52840</title>
      <text>Seeed XIAO nRF52840 — remplacer par la vraie part Seeed si disponible.
Broches utilisées :
  D2 (P0.02) = Crankshaft VRS
  D5 (P0.05) = ABS Wheel (via TLP291)
  A2         = Gear position ADC
  3V3 / GND  = alimentation</text>
      <views>
        <breadboardView>
          <geometry x="{xiao_x}" y="{xiao_y}" z="2.0" transform="1,0,0,1,0,0"/>
        </breadboardView>
      </views>
    </instance>"""

# 3. TLP291  → IC DIP-8 générique
tlp_x, tlp_y = bpos(19, 'b')
instances_xml += f"""
    <instance moduleIdRef="generic_ic_dip_8_300mil" modelIndex="3" path="core/generic_ic_dip_8.fzp">
      <title>IC1 TLP291</title>
      <text>TLP291 Opto-coupler DIP-8
Pin1=Anode(LED+) Pin2=Cathode(LED-) Pin4=GND Pin5=GND
Pin6=Emitter Pin7=Collector Pin3,8=NC</text>
      <views>
        <breadboardView>
          <geometry x="{tlp_x}" y="{tlp_y}" z="2.0" transform="1,0,0,1,0,0"/>
        </breadboardView>
      </views>
      <properties>
        <property name="chip label">TLP291</property>
        <property name="package">DIP8</property>
      </properties>
    </instance>"""

# 4. R1 100Ω  — entrée ABS → pin1 TLP291
instances_xml += inst_resistor(4,  "R1", "100\u03a9 \u2014 limitation courant LED TLP291", "100\u03a9", r1_x, r1_y)

# 5. R2 10kΩ  — pull-up 3.3V sur Collector (pin7 TLP291)
instances_xml += inst_resistor(5,  "R2", "10k\u03a9 \u2014 pull-up 3.3V sur Collector TLP291", "10k\u03a9", r2_x, r2_y)

# 6. R3 10kΩ  — pull-down GPIO P0.05
instances_xml += inst_resistor(6,  "R3", "10k\u03a9 \u2014 pull-down GPIO P0.05 (optionnel)", "10k\u03a9", r3_x, r3_y)

# 7. C1 100nF — découplage côté photo (Collector–GND)
instances_xml += inst_cap_ceramic(7, "C1", "100nF \u2014 d\u00e9couplage c\u00f4t\u00e9 photo TLP291", "100nF", c1_x, c1_y)

# 8. C2 10µF  — découplage côté LED (alimentation entrée ABS)
instances_xml += inst_cap_elec(8, "C2", "10\u00b5F \u2014 d\u00e9couplage c\u00f4t\u00e9 LED (entr\u00e9e ABS)", "10\u00b5F", "50V", c2_x, c2_y)

# 9. D1 1N4148 — protection polarité inverse sur signal vers GPIO
instances_xml += inst_diode(9, "D1", "1N4148 \u2014 protection polarit\u00e9 inverse signal GPIO P0.05", "1N4148", d1_x, d1_y)

# ── Fils principaux (couleurs = convention Fritzing) ─────────────────────────
wire_idx = 20
wires = [
    # Rouge = 3.3V : 3V3 rail → R2 haut
    (bpos(2, 'a')[0], bpos(2, 'a')[1], r2_x, r2_y - HOLE_STEP, "#cc0000"),
    # Noir = GND : GND rail → TLP291 pin2 (cathode)
    (bpos(2, 'j')[0], bpos(2, 'j')[1], tlp_pin2_x, tlp_pin2_y, "#000000"),
    # Noir = GND : GND rail → TLP291 pin4 (GND photo)
    (bpos(2, 'j')[0], bpos(2, 'j')[1], tlp_pin4_x, tlp_pin4_y, "#000000"),
    # Noir = GND : R3 bas → GND
    (r3_x, r3_y + HOLE_STEP, bpos(2, 'j')[0], bpos(2, 'j')[1], "#000000"),
    # Jaune = Signal ABS in → R1
    (bpos(14, 'b')[0], bpos(14, 'b')[1], r1_x, r1_y - HOLE_STEP, "#efef00"),
    # Orange = R1 → TLP291 pin1 (Anode)
    (r1_x, r1_y + HOLE_STEP, tlp_pin1_x, tlp_pin1_y, "#e85b00"),
    # Bleu = R2 bas → TLP291 Collector (pin7)
    (r2_x, r2_y + HOLE_STEP, tlp_pin7_x, tlp_pin7_y, "#0000ff"),
    # Bleu = TLP291 Collector → C1 haut
    (tlp_pin7_x, tlp_pin7_y, c1_x, c1_y - HOLE_STEP, "#0000ff"),
    # Bleu = C1 bas → GND
    (c1_x, c1_y + HOLE_STEP, bpos(2, 'j')[0], bpos(2, 'j')[1], "#000000"),
    # Vert = TLP291 Collector → D1 Anode
    (tlp_pin7_x, tlp_pin7_y, d1_x, d1_y - HOLE_STEP, "#00be00"),
    # Vert = D1 Cathode → GPIO D5 (P0.05) sur XIAO (col 10 ~ D5)
    (d1_x, d1_y + HOLE_STEP, bpos(10, 'b')[0], bpos(10, 'b')[1], "#00be00"),
    # Vert = R3 haut → D1 Cathode node
    (r3_x, r3_y - HOLE_STEP, d1_x, d1_y + HOLE_STEP, "#00be00"),
    # Violet = C2 : alimentation côté LED
    (bpos(14, 'b')[0], bpos(14, 'b')[1], c2_x, c2_y - HOLE_STEP, "#9b59b6"),
    (c2_x, c2_y + HOLE_STEP, bpos(2, 'j')[0], bpos(2, 'j')[1], "#000000"),
]

for (wx1, wy1, wx2, wy2, wcolor) in wires:
    instances_xml += inst_wire(wire_idx, wx1, wy1, wx2, wy2, wcolor)
    wire_idx += 1

# ── Notes textuelles ─────────────────────────────────────────────────────────
instances_xml += inst_label(60, "Note-ABS",
    "ENTRÉE ABS GSF650 (5V)\nSuperSeal 2-pin jaune\nPin1=Signal  Pin2=GND",
    bpos(13, 'a')[0] - 10, BOARD_Y - 60)

instances_xml += inst_label(61, "Note-VRS",
    "VRS Crankshaft → D2 (P0.02)\ndirect, pas d'isolateur",
    bpos(5, 'j')[0], BOARD_Y + 280)

instances_xml += inst_label(62, "Note-GEAR",
    "Capteur boîte → A2\nDiviseur de tension 0-3.3V\n(voir schéma principal)",
    bpos(9, 'j')[0], BOARD_Y + 280)

instances_xml += inst_label(63, "Note-GPIO",
    "Signal isolé → GPIO P0.05 (D5)\nVia TLP291 + D1 1N4148",
    bpos(28, 'a')[0] - 5, BOARD_Y - 60)


# ──────────────────────────────────────────────────────────────────────────────
# Assemblage du XML complet
# ──────────────────────────────────────────────────────────────────────────────

sketch_xml = f"""<?xml version='1.0' encoding='UTF-8'?>
<module fritzingVersion="0.9.9" moduleId="BanditBLE_GSF650_{uuid.uuid4().hex[:8]}">
  <title>BanditBLE GSF650 — ABS Opto-Isolateur</title>
  <date>2026-07-06</date>
  <author>BanditBLE</author>
  <description>Montage Seeed XIAO nRF52840 avec TLP291 pour isolation galvanique capteur ABS roue arrière GSF650.
Broches : D2=Crankshaft VRS | D5=ABS Wheel (opto) | A2=Gear ADC</description>
  <views>
    <breadboardView showGrid="1" alignToGrid="1">
      <layers image="breadboard/breadboard_blueprint.svg">
        <layer layerId="breadboard"/>
      </layers>
    </breadboardView>
    <schematicView showGrid="1" alignToGrid="1">
      <layers image="schematic/schematic_blueprint.svg">
        <layer layerId="schematic"/>
      </layers>
    </schematicView>
    <pcbView showGrid="1" alignToGrid="1">
      <layers image="pcb/pcb_blueprint.svg">
        <layer layerId="copper1" visible="false"/>
        <layer layerId="silkscreen"/>
        <layer layerId="copper0"/>
      </layers>
    </pcbView>
  </views>
  <instances>{instances_xml}
  </instances>
</module>
"""


# ──────────────────────────────────────────────────────────────────────────────
# Création du fichier .fzz (ZIP)
# ──────────────────────────────────────────────────────────────────────────────

output_path = r"c:\rix\BanditBLE_NRF52\BanditBLE_GSF650.fzz"

with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as zf:
    zf.writestr("BanditBLE_GSF650.fz", sketch_xml.encode("utf-8"))

print(f"✓ Fichier créé : {output_path}")
print(f"  Taille : {__import__('os').path.getsize(output_path)} octets")
print()
print("Notes d'import dans Fritzing :")
print("  • 'U0 XIAO-nRF52840' utilise Arduino Nano comme placeholder —")
print("    remplacer par la part officielle Seeed XIAO nRF52840")
print("    (disponible sur https://www.seeedstudio.com/blog/2021/06/16/seeed-fritzing-parts/)")
print("  • 'IC1 TLP291' utilise un IC DIP-8 générique —")
print("    remplacer par la vraie part TLP291 si disponible")
print("  • Les fils représentent les connexions principales ; vérifier")
print("    que chaque composant est bien inséré dans les bons trous")
