# Copyright (c) 2025 IBM
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from __future__ import annotations
from rdkit import Chem
import cppimport.import_hook  # noqa: F401
import ghl_chem
import os
import time
from pathlib import Path
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from graph_hook_library.ghl_bindings import apply_isomorphism


def _bond_order(bond: Chem.rdchem.Bond) -> int:
    """Return a stable integral representation for the bond type."""
    return int(bond.GetBondType())


def _mol_to_chem_graph(mol: Chem.Mol) -> ghl_chem.ChemGraph:
    """Convert an RDKit molecule into a ChemGraph instance."""
    graph = ghl_chem.ChemGraph()
    atom_map = {}
    for atom in mol.GetAtoms():
        vertex = graph.add_vertex(
            ghl_chem.Atom(
                graph.num_vertices(),
                atom.GetSymbol(),
                atom.GetAtomicNum(),
                atom.GetFormalCharge(),
                atom.GetIsAromatic(),
            )
        )
        atom_map[atom.GetIdx()] = vertex

    for bond in mol.GetBonds():
        atom_0 = atom_map[bond.GetBeginAtomIdx()]
        atom_1 = atom_map[bond.GetEndAtomIdx()]
        order = _bond_order(bond)
        graph.add_edge(atom_0, atom_1, ghl_chem.Bond(order))
    return graph


def match_with_rdkit(patt, file_name):
    matches = []
    total_loop = 0
    total_atoms = 0
    with Chem.SDMolSupplier(file_name) as suppl:
        loop_start = time.perf_counter()
        mol_count = 0
        for mol in suppl:
            mol_count += 1
            if mol is None:
                continue
            total_atoms += mol.GetNumAtoms()
            if mol.HasSubstructMatch(patt):
                matches.append(mol)
        total_loop = time.perf_counter() - loop_start
        print(f"Num RDKit matches: {len(matches)}")
        if mol_count:
            print(
                f"RDKit loop processed {mol_count} molecules in {total_loop:.3f}s "
                f"(avg {total_loop / mol_count:.6f}s)"
            )
    mean_atoms = total_atoms / mol_count if mol_count else 0
    print(f"Mean atoms per molecule: {mean_atoms:.2f}")
    return len(matches), total_loop, mean_atoms


def build_molecule_graphs(file_name):
    graphs = []
    with Chem.SDMolSupplier(file_name) as suppl:
        for mol in suppl:
            if mol is None:
                continue
            graphs.append(_mol_to_chem_graph(mol))
    return graphs


def match_with_ghl(patt, file_name, use_vf3: bool, plot_results: bool):
    mols = build_molecule_graphs(file_name)
    pattern_graph = _mol_to_chem_graph(patt)
    pattern_graph.write_graph(os.path.join("results", "pattern"), True)
    matches = []
    discover_loop_start = time.perf_counter()
    mode = "VF3" if use_vf3 else "VF2"
    iso = ghl_chem.SubStruct(pattern_graph.graph())
    for mol_idx, mol in enumerate(mols):
        discovered = iso.discover(
            pattern_graph.graph(),
            mol.graph(),
            first=True,
            nonoverlapping=True,
            use_vf3=use_vf3,
        )
        if discovered:
            matches.append(mol_idx)
    discover_loop_end = time.perf_counter()
    discover_loop_time = discover_loop_end - discover_loop_start
    print(
        f"GHL ({mode}) filtered {len(mols)} molecules in {discover_loop_time:.3f}s, finding {len(matches)} matches"
        f"(avg {discover_loop_time / len(mols):.6f}s)"
    )
    highlight_loop_start = time.perf_counter()
    highlighted_mols = []
    for mol_idx, mol in enumerate(mols):
        discovered = apply_isomorphism(mol, iso, True, use_vf3)
        if discovered:
            highlighted_mols.append(mol)
    highlight_loop_end = time.perf_counter()
    highlight_loop_time = highlight_loop_end - highlight_loop_start
    if plot_results:
        for mol_idx, mol in enumerate(highlighted_mols):
            mol.write_graph(os.path.join("results", f"mol_{mol_idx}"), True)
    print(
        f"GHL ({mode}) highlighted {len(matches)} molecules in {highlight_loop_time:.3f}s "
        f"(avg {highlight_loop_time / len(mols):.6f}s)"
    )
    return len(matches), discover_loop_time, highlight_loop_time


def configure_plot_style():
    plt.style.use("default")
    cmap = plt.cm.viridis
    plt.rcParams.update(
        {
            "axes.prop_cycle": mpl.cycler(
                color=[cmap(i) for i in np.linspace(0.1, 1, 6)],
            ),
            "font.family": "serif",
            "font.serif": ["DejaVu Serif", "CMU Serif", "Times New Roman"],
            "font.size": 15,
        }
    )


def plot_results(pattern_labels, rdkit_times, ghl_times):
    indices = list(range(len(pattern_labels)))
    datasets = [("RDKit Filter", rdkit_times)]
    for use_vf3 in (True, False):
        suffix = "VF3" if use_vf3 else "VF2"
        datasets.append((f"GHL Filter {suffix}", ghl_times[use_vf3]["discover"]))
        datasets.append((f"GHL Highlight {suffix}", ghl_times[use_vf3]["highlight"]))

    total_width = 0.85
    bar_width = total_width / len(datasets)
    fig, ax = plt.subplots(figsize=(14, 3.8))
    for idx, (label, values) in enumerate(datasets):
        offset = -total_width / 2 + idx * bar_width + bar_width / 2
        positions = [i + offset for i in indices]
        ax.bar(
            positions,
            values,
            width=bar_width,
            label=label,
            edgecolor="#0f172a",
            linewidth=0.7,
            alpha=0.9,
        )

    ax.set_xticks(indices)
    ax.set_xticklabels(pattern_labels, rotation=15, ha="right")
    ax.set_ylabel("Time (s)")
    ax.set_xlabel("SMARTS Pattern")
    ax.set_title("Molecular Substructure Latency", loc="left")
    ax.set_axisbelow(True)
    ax.legend(loc="upper right", ncol=2, columnspacing=0.8, borderpad=0.6)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    fig.tight_layout()
    plot_path = os.path.join("results", "rdkit.pdf")
    plt.savefig(plot_path, dpi=220)
    print(f"Saved benchmark plot to {plot_path}")


def main():
    configure_plot_style()
    patterns = [
        "c1ccc(Cl)cc1",
        "[CX3]=[OX1]",
        "[N+](=O)[O-]",
        "cN(C)C",
        "[S](=O)(=O)N",
        "c1ncccc1",
    ]
    file_name = "molecules.sdf"
    rdkit_times = []
    ghl_times = {
        False: {"discover": [], "highlight": []},
        True: {"discover": [], "highlight": []},
    }
    pattern_labels = []
    mean_atoms_across_runs = []
    for string_pattern in patterns:
        patt = Chem.MolFromSmarts(string_pattern)
        rdkit_matches, rdkit_time, mean_atoms = match_with_rdkit(patt, file_name)
        pattern_labels.append(string_pattern)
        rdkit_times.append(rdkit_time)
        mean_atoms_across_runs.append(mean_atoms)
        for use_vf3 in (True, False):
            ghl_chem_matches, discover_time, highlight_time = match_with_ghl(
                patt, file_name, use_vf3, string_pattern == "c1ncccc1"
            )
            assert rdkit_matches == ghl_chem_matches
            ghl_times[use_vf3]["discover"].append(discover_time)
            ghl_times[use_vf3]["highlight"].append(highlight_time)
    plot_results(pattern_labels, rdkit_times, ghl_times)
    if mean_atoms_across_runs:
        overall_mean_atoms = sum(mean_atoms_across_runs) / len(mean_atoms_across_runs)
        print(f"Average atoms per molecule across patterns: {overall_mean_atoms:.2f}")


if __name__ == "__main__":
    os.chdir(Path(__file__).parent)
    os.makedirs("results", exist_ok=True)
    main()
