"""Make build/reference.docx: pandoc's default reference document with the code font at 9 pt.

pandoc's default sets the code character style (Verbatim Char, Consolas) at 11 pt, which fits
only about 77 columns in the 6.5 in text width of a US Letter page with 1 in margins; a wrapped
78th character then appears alone at the left margin of the next line. At 9 pt about 94 columns
fit, which covers the 76-column help text and the wrapped command lines of the procedure.

Run from this directory:  python3 make_reference_docx.py
build_docx.sh passes the result to pandoc as --reference-doc.
"""
import pathlib
import re
import subprocess
import zipfile

import pypandoc

CODE_FONT_HALF_POINTS = "18"
"""Font size of code in Word's half-point units: 18 = 9 pt."""
CODE_STYLE_IDS = ("VerbatimChar", "SourceCode")
"""The character style of inline and block code, and the block paragraph style if present."""
OUTPUT = pathlib.Path(__file__).resolve().parent / "reference.docx"


def set_code_font_size(styles_xml: str) -> tuple[str, int]:
    """Set the font size of the code styles; returns the new XML and how many styles changed."""
    changed = 0
    for style_id in CODE_STYLE_IDS:
        pattern = re.compile(r'(<w:style [^>]*w:styleId="%s"[^>]*>.*?)</w:style>' % style_id, re.S)
        match = pattern.search(styles_xml)
        if match is None:
            continue
        body = match.group(1)
        size = '<w:sz w:val="%s"/>' % CODE_FONT_HALF_POINTS
        if "<w:sz " in body:
            body = re.sub(r'<w:sz w:val="\d+"\s*/>', size, body)
        elif "<w:rPr>" in body:
            body = body.replace("<w:rPr>", "<w:rPr>" + size, 1)
        else:
            body = body + "<w:rPr>" + size + "</w:rPr>"
        styles_xml = styles_xml[:match.start(1)] + body + styles_xml[match.end(1):]
        changed += 1
    return styles_xml, changed


def main() -> None:
    pandoc = pypandoc.get_pandoc_path()
    default = subprocess.run([pandoc, "--print-default-data-file", "reference.docx"],
                             capture_output=True, check=True).stdout
    source = OUTPUT.with_name("reference.default.docx")
    source.write_bytes(default)
    with zipfile.ZipFile(source) as zin, zipfile.ZipFile(OUTPUT, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if item.filename == "word/styles.xml":
                text, changed = set_code_font_size(data.decode("utf-8"))
                print(f"code styles changed: {changed}")
                data = text.encode("utf-8")
            zout.writestr(item, data)
    source.unlink()
    print(f"wrote {OUTPUT}")


if __name__ == "__main__":
    main()
