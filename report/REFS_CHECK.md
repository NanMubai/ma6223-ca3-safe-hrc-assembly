# REFS_CHECK — bibliography verification

Method: every DOI in `refs.bib` was resolved with
`curl -s https://api.crossref.org/works/<DOI>` on **2026-11-15**; the fields in
`refs.bib` (authors, title, venue, year, volume/issue, pages) were taken from the
returned Crossref metadata. "Match?" compares the Crossref record with the entry
requested in the task brief.

| Key | DOI | Crossref title found | Match? |
|---|---|---|---|
| `hogan1985impedance` | 10.1115/1.3140702 | Impedance Control: An Approach to Manipulation: Part I—Theory | yes — J. Dyn. Sys., Meas., Control 107(1):1–7, 1985 |
| `hogan1985impedanceII` | 10.1115/1.3140713 | Impedance Control: An Approach to Manipulation: Part II—Implementation | yes — verified DOI (not given in brief), 107(1):8–16 |
| `hogan1985impedanceIII` | 10.1115/1.3140701 | Impedance Control: An Approach to Manipulation: Part III—Applications | yes — verified DOI (not given in brief), 107(1):17–24 |
| `khatib1987unified` | 10.1109/JRA.1987.1087068 | A unified approach for motion and force control of robot manipulators: The operational space formulation | yes — IEEE J. Robot. Autom. 3(1):43–53, 1987 |
| `calinon2007learning` | 10.1109/TSMCB.2006.886952 | On Learning, Representing, and Generalizing a Task in a Humanoid Robot | yes — IEEE Trans. SMC-B 37(2):286–298, 2007 |
| `kana2023boxinbox` | 10.3390/s23218721 | Learning Compliant Box-in-Box Insertion through Haptic-Based Robotic Teleoperation | yes — Sensors 23(21):8721, 2023, 6 authors as in Crossref |
| `todorov2012mujoco` | 10.1109/IROS.2012.6386109 | MuJoCo: A physics engine for model-based control | yes — IROS 2012, pp. 5026–5033 |
| `haddadin2017collision` | 10.1109/TRO.2017.2723903 | Robot Collisions: A Survey on Detection, Isolation, and Identification | yes — IEEE T-RO 33(6):1292–1312, 2017; Crossref spells the author "Albu-Schaffer", `refs.bib` uses "Albu-Schäffer" (same person, correct diacritic) |
| `ferraguti2013tank` | 10.1109/ICRA.2013.6631284 | A tank-based approach to impedance control with variable stiffness | yes after correction — brief DOI 10.1109/ICRA.2013.6631141 resolves to *Geravand, Flacco, De Luca, "Human-robot physical interaction and collaboration using an industrial robot with a closed control architecture"*, i.e. a **mismatch**; replaced by the verified Ferraguti/Secchi/Fantuzzi DOI (ICRA 2013, pp. 4948–4953) |
| `iso15066` | — (standards are not in Crossref) | — | n/a — ISO/TS 15066:2016, *Robots and robotic devices — Collaborative robots*; metadata from the ISO catalogue, no DOI exists |
| `iso10218` | — (not in Crossref) | — | n/a — ISO 10218-1:2011, *Safety requirements for industrial robots — Part 1: Robots* |
| `iso10218part2` | — (not in Crossref) | — | n/a — ISO 10218-2:2011, *Safety requirements for industrial robots — Part 2: Robot systems and integration* (split from the brief's single `iso10218` item so both parts stay separately citable) |
| `menagerie2022` | — (software, not in Crossref) | — | n/a — GitHub `google-deepmind/mujoco_menagerie`; commit recorded in the entry is `370c43965323c0944b433b202aac5058b5f907a9` (matches `assets/franka_emika_panda/MENAGERIE_COMMIT`) |
| `frankadocs` | — (vendor documentation, no DOI) | — | n/a — <https://frankaemika.github.io/docs/>, accessed 2026-11-15 |

## Rejected / corrected DOIs

| DOI from brief | Resolves to | Action |
|---|---|---|
| 10.1109/ICRA.2013.6631141 | Geravand, Flacco, De Luca, "Human-robot physical interaction and collaboration using an industrial robot with a closed control architecture", ICRA 2013 | rejected; replaced by **10.1109/ICRA.2013.6631284** (Ferraguti, Secchi, Fantuzzi) |
| 10.1115/1.3140700 (candidate for Hogan Part II) | Lyou & Bien, "Decentralized Adaptive Stabilization…", J. Dyn. Sys., Meas., Control 107(1) | rejected; Part II is **10.1115/1.3140713**, Part III is **10.1115/1.3140701** |

All nine DOI-bearing entries resolve (`status = ok` in Crossref); none besides the
Ferraguti entry was mismatched.
