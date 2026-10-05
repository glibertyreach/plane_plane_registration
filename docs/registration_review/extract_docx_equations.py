"""Linearize a docx body: paragraphs (with style + list level), tables, OMML math, footnotes."""
import sys, re
import xml.etree.ElementTree as ET
NS={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
    'm':'http://schemas.openxmlformats.org/officeDocument/2006/math'}
W='{%s}'%NS['w']; M='{%s}'%NS['m']
def chr_of(pr,tag,d):
    if pr is None: return d
    c=pr.find(M+tag)
    if c is None: return d
    v=c.get(M+'val'); return v if v is not None else d
def omml(e):
    t=e.tag
    if t==M+'r': return ''.join((x.text or '') for x in e.findall(M+'t'))
    if t==M+'t': return e.text or ''
    if t in (M+'oMath',M+'oMathPara',M+'e',M+'num',M+'den',M+'sub',M+'sup',M+'deg',M+'fName',M+'lim'):
        return ''.join(omml(c) for c in e)
    if t==M+'f': return '(%s)/(%s)'%(omml(e.find(M+'num')),omml(e.find(M+'den')))
    if t==M+'sSub': return '%s_{%s}'%(omml(e.find(M+'e')),omml(e.find(M+'sub')))
    if t==M+'sSup': return '%s^{%s}'%(omml(e.find(M+'e')),omml(e.find(M+'sup')))
    if t==M+'sSubSup': return '%s_{%s}^{%s}'%(omml(e.find(M+'e')),omml(e.find(M+'sub')),omml(e.find(M+'sup')))
    if t==M+'d':
        pr=e.find(M+'dPr'); b=chr_of(pr,'begChr','('); en=chr_of(pr,'endChr',')'); sep=chr_of(pr,'sepChr','|')
        return b+sep.join(omml(c) for c in e.findall(M+'e'))+en
    if t==M+'nary':
        pr=e.find(M+'naryPr'); c=chr_of(pr,'chr','∫')
        sb=e.find(M+'sub'); sp=e.find(M+'sup')
        return '%s_{%s}^{%s}[%s]'%(c,omml(sb) if sb is not None else '',omml(sp) if sp is not None else '',omml(e.find(M+'e')))
    if t==M+'acc':
        pr=e.find(M+'accPr'); c=chr_of(pr,'chr','̂'); return '\\acc(%s){%s}'%(c,omml(e.find(M+'e')))
    if t==M+'bar': return '\\bar{%s}'%omml(e.find(M+'e'))
    if t==M+'rad':
        dg=e.find(M+'deg'); d=omml(dg) if dg is not None else ''
        return '\\sqrt%s{%s}'%('[%s]'%d if d else '',omml(e.find(M+'e')))
    if t==M+'m':
        rows=[' & '.join(omml(c) for c in r.findall(M+'e')) for r in e.findall(M+'mr')]
        return '[' + ' \\\\ '.join(rows) + ']'
    if t==M+'func': return '%s(%s)'%(omml(e.find(M+'fName')),omml(e.find(M+'e')))
    if t==M+'limLow': return '%s_{%s}'%(omml(e.find(M+'e')),omml(e.find(M+'lim')))
    if t==M+'groupChr': return '\\group{%s}'%omml(e.find(M+'e'))
    if t==M+'eqArr': return ' ; '.join(omml(c) for c in e.findall(M+'e'))
    if t==M+'box' or t==M+'borderBox': return omml(e.find(M+'e'))
    if t.endswith('Pr'): return ''
    return ''.join(omml(c) for c in e)
def para_text(p):
    out=[]
    for ch in p:
        if ch.tag==W+'r':
            for x in ch:
                if x.tag==W+'t': out.append(x.text or '')
                elif x.tag==W+'tab': out.append('\t')
                elif x.tag==W+'br': out.append('\n')
                elif x.tag==W+'sym': out.append('[sym %s]'%x.get(W+'char'))
                elif x.tag==W+'footnoteReference': out.append('[fn%s]'%x.get(W+'id'))
                elif x.tag==W+'drawing': out.append('[DRAWING]')
                elif x.tag==W+'object': out.append('[OBJECT]')
        elif ch.tag in (M+'oMath',M+'oMathPara'):
            out.append(' $'+omml(ch)+'$ ')
        elif ch.tag==W+'hyperlink' or ch.tag==W+'smartTag' or ch.tag==W+'ins' or ch.tag==W+'sdt':
            out.append(para_text(ch))
        elif ch.tag==W+'del':
            out.append('[DEL:'+para_text(ch)+']')
    return ''.join(out)
def para_meta(p):
    pPr=p.find(W+'pPr'); style=''; lvl=None; numid=None
    if pPr is not None:
        s=pPr.find(W+'pStyle');
        if s is not None: style=s.get(W+'val')
        n=pPr.find(W+'numPr')
        if n is not None:
            l=n.find(W+'ilvl'); i=n.find(W+'numId')
            lvl=l.get(W+'val') if l is not None else '0'; numid=i.get(W+'val') if i is not None else '?'
    return style,lvl,numid
def walk(body,depth=0):
    for el in body:
        if el.tag==W+'p':
            style,lvl,numid=para_meta(el); txt=para_text(el)
            tag=style or 'p'
            if lvl is not None: tag+=' L%s#%s'%(lvl,numid)
            print('  '*depth+'[%s] %s'%(tag,txt))
        elif el.tag==W+'tbl':
            print('  '*depth+'[TABLE]')
            for r in el.findall(W+'tr'):
                cells=[]
                for c in r.findall(W+'tc'):
                    cells.append(' / '.join(para_text(p) for p in c.iter(W+'p')))
                print('  '*depth+' | '+' | '.join(cells))
            print('  '*depth+'[/TABLE]')
        elif el.tag==W+'sdt':
            print('  '*depth+'[SDT]'); walk(el.find(W+'sdtContent'),depth+1)
root=ET.parse(sys.argv[1]).getroot()
walk(root.find(W+'body'))
