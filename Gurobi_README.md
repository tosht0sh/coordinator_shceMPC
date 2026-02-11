

TrajPlan-ScheMPC setup summary (Ubuntu 22.04 + external SSD /mnt/intenso)


Goal
- Run the TrajPlan-ScheMPC simulation locally on Ubuntu.
- Install and license Gurobi, install Rust toolchain, install Python deps (including opengen), and fix plotting so the GUI works.

NOTE: Chose directory of your choice :D

--------------------------------------------------------------------------------
1) Gurobi Optimizer install (Linux tar.gz)
- Download: gurobi13.0.1_linux64.tar.gz
- Extract and move to /opt:
  cd ~/Downloads
  tar -xvf gurobi13.0.1_linux64.tar.gz
  sudo mkdir -p /opt
  sudo mv gurobi1301 /opt/

- Add env vars to ~/.bashrc:
  cat >> ~/.bashrc <<'EOF'
  export GUROBI_HOME="/opt/gurobi1301/linux64"
  export PATH="$GUROBI_HOME/bin:$PATH"
  export LD_LIBRARY_PATH="$GUROBI_HOME/lib:$LD_LIBRARY_PATH"
  EOF

  Then:
  source ~/.bashrc

- Verify:
  gurobi_cl --version
  which grbgetkey

--------------------------------------------------------------------------------
2) Gurobi license activation (requires Chalmers VPN / university network)
- At this step you needed the university network or a FULL-TUNNEL VPN, otherwise
  grbgetkey fails with “not recognized as academic domain”.
- Just request a new academic licence (valid for one year), since the previous one in installed on Widows. 
It’s that a license is tied to a machine (so you typically need a new one for a new OS install / machine id).

- Chalmers VPN (eduVPN) link used:
  https://docs.eduvpn.org/client/linux/installation.html

- Once VPN was full-tunnel, you request a NEW license and run:
  grbgetkey <NEW-KEY>

- Stored license file at:
  ~/gurobi.lic

- Verify license:
  gurobi_cl --license


--------------------------------------------------------------------------------
3) Rust toolchain (needed by opengen for codegen/build)
- Install Rust via rustup:
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  source "$HOME/.cargo/env"

- Verify:
  cargo --version
  rustc --version

--------------------------------------------------------------------------------
4) System build deps (needed by opengen & some Python packages)
- Install:
  sudo apt update
  sudo apt install -y build-essential clang python3-venv

--------------------------------------------------------------------------------
5) Python venv inside the repo (one-time creation)
- Create a virtual environment inside the repo so everything stays self-contained:
  cd /mnt/intenso/code/dev/TrajPlan-ScheMPC
  python3 -m venv .venv
  source .venv/bin/activate
  pip install -U pip
  pip install -r requirements.txt

Sanity checks:
  python -c "import gurobipy as gp; gp.Model(); print('gurobipy OK')"
  python -c "import opengen as og; print('opengen OK')"

--------------------------------------------------------------------------------
6) Build the OpEn MPC solver (one-time unless you change solver config)
- Build solver module:
  python src/build_solver.py

This generates the compiled solver bindings under:
  mpc_solver/navi_fast/...

--------------------------------------------------------------------------------
7) Fix plotting interactivity (Matplotlib backend)
Symptom:
- Running main produced warnings:
  “FigureCanvasAgg is non-interactive, and thus cannot be shown”
- The visualizer waits for button/key press; without an interactive backend it
  appears “stuck”.

Fix:
- Install Tk support:
  sudo apt update
  sudo apt install -y python3-tk

--------------------------------------------------------------------------------
8) Run the simulation
- With venv active:
  cd /mnt/intenso/code/dev/TrajPlan-ScheMPC
  source .venv/bin/activate
  python src/main.py

(You should now see the interactive figure with Play/Pause and the run progresses.)

--------------------------------------------------------------------------------
