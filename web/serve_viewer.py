#!/usr/bin/env python3
"""Serves the RaiSim browser viewer with the demo's robot files stored at the server's own paths.

A browser cannot read the meshes the server streams by path, so the bundle's viewer page gets a few
lines that write rsc/raiway into the viewer's file system under /examples/rsc/raiway, the only place its
file writer accepts, and start the viewer with that directory as a resource directory, where streamed
meshes and the shared scene files are looked up by their path tails.
The bundle (webassembly-<version>.zip, unzipped) is left untouched: its files are copied into a work
directory holding the patched viewer.mjs, the file list and the robot files, and its serve.py serves that
(it refuses anything that resolves outside the served directory, so links would not do).
"""
import argparse
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
RSC = os.path.realpath(os.path.join(HERE, "../rsc/raiway"))
VIEWER_FILES = "/examples/rsc/raiway"
HOOK = """  // Raiway demo: its robot files, found through the resource directory below (web/serve_viewer.py)
  try {
    const extra = await (await fetchOk(new URL('./raiway-files.json', import.meta.url), 'the Raiway file list')).json();
    for (const {path, url} of extra)
      writeModuleFile(viewer, viewer._viewer_write_file, path,
                      await (await fetchOk(new URL(url, import.meta.url), path)).arrayBuffer());
  } catch (error) {
    console.warn('Raiway files were not stored:', error);
  }
"""
ANCHOR = "  progress.done();\n  byId('cache-status').textContent"
MAIN_CALL = "await viewer.callMain(viewerArguments(location.search));"
MAIN_HOOK = "await viewer.callMain([...viewerArguments(location.search), '--resource-dir', '%s']);" % VIEWER_FILES


def copy_tree(source, out, skip=()):
    for folder, dirs, names in os.walk(source):
        dirs[:] = [d for d in dirs if d not in skip]
        relative = os.path.relpath(folder, source)
        os.makedirs(os.path.join(out, relative), exist_ok=True)
        for name in names:
            if name in skip and relative == ".":
                continue
            src, dst = os.path.join(folder, name), os.path.join(out, relative, name)
            if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(src) or os.path.getsize(dst) != os.path.getsize(src):
                shutil.copy2(src, dst)


def find_bundle():
    parent = os.path.realpath(os.path.join(HERE, "../.."))
    candidates = sorted(d for d in os.listdir(parent) if d.startswith("webassembly-") and os.path.isdir(os.path.join(parent, d, "web-examples")))
    return os.path.join(parent, candidates[-1]) if candidates else None


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--bundle", default=find_bundle(), help="unzipped webassembly-<version> directory (default: newest next to this repository)")
    parser.add_argument("--out", default=os.path.join(HERE, "../build/web"), help="work directory for the patched copy")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    if not args.bundle or not os.path.isdir(os.path.join(args.bundle, "web-examples")):
        sys.exit("no WebAssembly bundle found; unzip webassembly-<version>.zip next to this repository or pass --bundle")

    source = os.path.join(args.bundle, "web-examples")
    out = os.path.join(os.path.realpath(args.out), "web-examples")
    copy_tree(source, out, skip=("viewer.mjs",))
    text = open(os.path.join(source, "viewer.mjs")).read()
    if ANCHOR not in text or MAIN_CALL not in text:
        sys.exit("this bundle's viewer.mjs has no known place for the hook; the patch needs updating")
    text = text.replace(ANCHOR, HOOK + ANCHOR, 1).replace(MAIN_CALL, MAIN_HOOK, 1)
    open(os.path.join(out, "viewer.mjs"), "w").write(text)

    copy_tree(RSC, os.path.join(out, "raiway"), skip=("policies",))
    files = []
    for folder, _, names in os.walk(os.path.join(out, "raiway")):
        for name in names:
            relative = os.path.relpath(os.path.join(folder, name), os.path.join(out, "raiway"))
            files.append({"path": VIEWER_FILES + "/" + relative, "url": "raiway/" + relative})
    json.dump(sorted(files, key=lambda f: f["path"]), open(os.path.join(out, "raiway-files.json"), "w"), indent=1)
    print("viewer at http://localhost:%d/web-examples/index.html with %d Raiway files" % (args.port, len(files)))
    os.execv(sys.executable, [sys.executable, os.path.join(args.bundle, "serve.py"), "--directory", os.path.dirname(out), "--port", str(args.port)])


if __name__ == "__main__":
    main()
