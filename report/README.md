# Report — Safe Human–Robot Collaborative Assembly

IEEE conference paper (IEEEtran, **A4**, **max 8 pages** including figures, tables
and references) for the course project on LfD + variable impedance control for
safe box-in-box assembly with a Franka Panda in MuJoCo.

## Build

```sh
cd report
latexmk -pdf main.tex          # full build (pdflatex + bibtex + reruns)
latexmk -C                     # clean all build artifacts
```

Log checks:

```sh
grep -i "undefined" main.log   # must print nothing
pdfinfo main.pdf | grep Pages  # must be <= 8
```

Requirements: TeX Live 2026 with `IEEEtran.cls` (already installed; nothing is
downloaded).

## Layout

```
report/
├── main.tex                  # preamble, title/author block, \input of sections
├── refs.bib                  # verified bibliography (see REFS_CHECK.md)
├── REFS_CHECK.md             # DOI ↔ Crossref verification table
├── figures/                  # put figures here (\graphicspath{{figures/}})
└── sections/
    ├── 00_abstract.tex
    ├── 01_introduction.tex            (I)
    ├── 02_related_work.tex            (II)
    ├── 03_requirements_architecture.tex (III)
    ├── 04_method.tex                  (IV, inputs IV-A…IV-C)
    ├── 04a_system_modeling.tex        (IV-A)
    ├── 04b_lfd.tex                    (IV-B)
    ├── 04c_vic.tex                    (IV-C)
    ├── 05_safety.tex                  (V)
    ├── 06_experiments.tex             (VI, subsections A–D)
    ├── 07_discussion.tex              (VII)
    └── 08_conclusion.tex              (VIII)
```

Every section file starts with `% Owner: <part> | Page budget: <x> pages`.
Placeholders are rendered in red via `\todo{...}`; remove them as text lands.

## Page budgets (total 8.0 pages)

| Part | Owner | Budget |
|---|---|---|
| Abstract + I | A | 0.75 |
| II Related Work | A | 0.75 |
| III Requirements and Architecture | B | 1.0 |
| IV-A System Modeling | C | 0.6 (≤ 1.25 column, ≤ 350 words, ≤ 5 equations, ≤ 1 figure + 1 table) |
| IV-B Learning from Demonstration | D | 0.7 |
| IV-C Variable Impedance Control | E | 0.7 |
| V Safety / ISO-TS 15066 | A | 1.0 |
| VI-A Setup | B | 0.4 |
| VI-B LfD Results | D | 0.5 |
| VI-C VIC Results | E | 0.6 |
| VI-D Safety Results | A | 0.25 |
| VII + VIII Discussion + Conclusion | A | 0.4 |
| References | all | 0.35 |
| **Total** | | **8.0** |

Budgets are enforced per part: check the column/page count after compiling and
split or trim text rather than borrowing from another section. The current
skeleton compiles to 2 pages because `\nocite{*}` forces all 14 references to
print; that reference overflow disappears once every entry is cited in text and
the `\nocite{*}` line (plus its comment) is removed.

## Rules

- **No section exceeds its budget**; the sum must stay ≤ 8 pages. A4 paper is
  fixed by the `a4paper` class option — do not remove it.
- **Figures are single-column** unless the whole team agrees to a two-column
  span; state the agreement in the PR/commit message. Use `figures/` and refer
  to files without a path (the `\graphicspath` handles it).
- Cite with `\cite{key}`; keys are listed in `REFS_CHECK.md`. Do not add a DOI
  that has not been checked against Crossref.
- No personal names or e-mail addresses anywhere: authors are `Member A` … `Member E`
  and the affiliation block is a placeholder. Keep it that way in camera-ready
  drafts until the course staff supplies the real data.
- Section IV-A (System Modeling) is the only section that will be written now;
  the others stay one-line placeholders until their owner fills them in.
