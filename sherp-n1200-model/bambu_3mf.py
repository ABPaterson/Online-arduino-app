"""
Write a Bambu Studio project 3MF (multi-plate, printer + filaments set) from
manifold3d solids.

Profiles are Bambu Lab's own, flattened from the BambuStudio repository
(resources/profiles/BBL). Point PROFILE_DIR at a checkout of that folder, or
leave it missing and a plain 3MF is written instead.

Layout rules used by Bambu Studio for multi-plate projects:
  plates sit in a grid, ceil-ish(sqrt(n)) columns, each plate offset by
  (bed size * 1.2) in X per column and -(bed size * 1.2) in Y per row.
"""
import json
import math
import os
import uuid
import zipfile
from xml.sax.saxutils import escape

import numpy as np

PROFILE_DIR = os.environ.get(
    "BBL_PROFILES", "/home/user/bambulab/bambustudio/resources/profiles/BBL")
APP_VERSION = "BambuStudio-02.02.00.85"
BED = 256.0
PLATE_GAP = 1.0 / 5.0

MACHINE = "Bambu Lab A1 0.4 nozzle"
PROCESS = "0.20mm Standard @BBL A1"
FILAMENTS = {  # extruder slot -> (profile, colour)
    1: ("Bambu PLA Basic @BBL A1", "#F2F2F2"),
    2: ("Bambu TPU 95A HF @BBL A1", "#161616"),
}
DROP_KEYS = {"inherits", "from", "instantiation", "setting_id", "name",
             "type", "description", "base_id", "filament_id_for_ams"}


def _flatten(kind, name):
    path = os.path.join(PROFILE_DIR, kind, name + ".json")
    j = json.load(open(path, encoding="utf-8"))
    parent = j.get("inherits")
    base = _flatten(kind, parent) if parent else {}
    base.update(j)
    return base


def profiles_available():
    return os.path.isfile(os.path.join(PROFILE_DIR, "machine", MACHINE + ".json"))


def project_config(overrides=None):
    m = _flatten("machine", MACHINE)
    p = _flatten("process", PROCESS)
    cfg = {}
    for src in (m, p):
        for k, v in src.items():
            if k not in DROP_KEYS:
                cfg[k] = v
    fil = [_flatten("filament", FILAMENTS[i][0]) for i in sorted(FILAMENTS)]
    fkeys = set().union(*[f.keys() for f in fil]) - DROP_KEYS
    for k in fkeys:
        vals = []
        for f in fil:
            v = f.get(k, fil[0].get(k))
            vals.extend(v if isinstance(v, list) else [v])
        cfg[k] = vals
    cfg["filament_settings_id"] = [FILAMENTS[i][0] for i in sorted(FILAMENTS)]
    cfg["filament_colour"] = [FILAMENTS[i][1] for i in sorted(FILAMENTS)]
    cfg["print_settings_id"] = PROCESS
    cfg["printer_settings_id"] = MACHINE
    cfg["printer_model"] = m.get("printer_model", "Bambu Lab A1")
    cfg["name"] = "project_settings"
    cfg["from"] = "project"
    if overrides:
        cfg.update(overrides)
    return cfg


def plate_origin(index, n_plates):
    v = math.sqrt(n_plates)
    cols = int(round(v)) + (1 if v > round(v) else 0)
    cols = max(cols, 1)
    col, row = index % cols, index // cols
    step = BED * (1 + PLATE_GAP)
    return col * step, -row * step


def _mesh_xml(man):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    f = np.asarray(mesh.tri_verts)
    out = ['<mesh><vertices>']
    out.extend('<vertex x="%.4f" y="%.4f" z="%.4f"/>' % tuple(p) for p in v)
    out.append('</vertices><triangles>')
    out.extend('<triangle v1="%d" v2="%d" v3="%d"/>' % tuple(t) for t in f)
    out.append('</triangles></mesh>')
    return "".join(out)


def write_project(path, plates, title, overrides=None):
    """plates: list of (plate_name, [(object_name, manifold_on_bed, extruder)])
    where each manifold is already positioned inside 0..256 on its plate."""
    n = len(plates)
    model = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
             'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021">',
             ' <metadata name="Application">%s</metadata>' % APP_VERSION,
             ' <metadata name="BambuStudio:3mfVersion">1</metadata>',
             ' <metadata name="Title">%s</metadata>' % escape(title),
             ' <metadata name="Designer">Macshaw / AirLoom (generated)</metadata>',
             ' <resources>']
    build, settings, plate_xml, assemble = [], [], [], []
    oid = 0
    for pi, (pname, objs) in enumerate(plates):
        ox, oy = plate_origin(pi, n)
        inst = []
        for name, man, extruder in objs:
            oid += 1
            model.append('  <object id="%d" name="%s" type="model">%s</object>'
                         % (oid, escape(name), _mesh_xml(man)))
            tf = "1 0 0 0 1 0 0 0 1 %.4f %.4f 0" % (ox, oy)
            build.append('  <item objectid="%d" transform="%s" printable="1"/>' % (oid, tf))
            settings.append(
                '  <object id="%d">\n'
                '    <metadata key="name" value="%s"/>\n'
                '    <metadata key="extruder" value="%d"/>\n'
                '    <part id="1" subtype="normal_part">\n'
                '      <metadata key="name" value="%s"/>\n'
                '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
                '    </part>\n'
                '  </object>' % (oid, escape(name), extruder, escape(name)))
            inst.append(oid)
            assemble.append('   <assemble_item object_id="%d" instance_id="0" '
                            'transform="%s" offset="0 0 0" />' % (oid, tf))
        mi = "\n".join(
            '    <model_instance>\n'
            '      <metadata key="object_id" value="%d"/>\n'
            '      <metadata key="instance_id" value="0"/>\n'
            '      <metadata key="identify_id" value="%d"/>\n'
            '    </model_instance>' % (o, 100 + o) for o in inst)
        plate_xml.append(
            '  <plate>\n'
            '    <metadata key="plater_id" value="%d"/>\n'
            '    <metadata key="plater_name" value="%s"/>\n'
            '    <metadata key="locked" value="false"/>\n'
            '%s\n'
            '  </plate>' % (pi + 1, escape(pname), mi))
    model.append(' </resources>')
    model.append(' <build>')
    model.extend(build)
    model.append(' </build>')
    model.append('</model>')

    model_settings = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n' +
                      "\n".join(settings) + "\n" + "\n".join(plate_xml) +
                      '\n  <assemble>\n' + "\n".join(assemble) + '\n  </assemble>\n</config>\n')
    ct = ('<?xml version="1.0" encoding="UTF-8"?>\n'
          '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
          '<Default Extension="png" ContentType="image/png"/>'
          '<Default Extension="config" ContentType="text/xml"/>'
          '</Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel-1" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", "\n".join(model))
        z.writestr("Metadata/model_settings.config", model_settings)
        if profiles_available():
            z.writestr("Metadata/project_settings.config",
                       json.dumps(project_config(overrides), indent=4))
