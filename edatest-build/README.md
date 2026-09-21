# edatest:1.0

Packages the supplied, unchanged Quan HFSM agent for spi_xfer_public. The supplied harness is included unchanged. The remaining DUT files, simulation tools, and precompiled models come from the local official eda-coverage-base:1.0 image.

Running harness.py directly selects its built-in RandAgent. This image instead starts run_agent.py, which explicitly uses policy=hfsm and backend=verilator. It passes the harness's public default parameters explicitly; this is a local functionality test, not a hidden evaluation score. It does not alter the HFSM algorithm.

## Build on Ubuntu

Extract this directory, open a terminal inside it, then run:

```bash
sudo docker image inspect eda-coverage-base:1.0
sudo docker build -t edatest:1.0 .
```

The official base image must already be loaded with docker load. No pip installation is required by the supplied Python files. Do not commit the base image archive to Git.

## Run without host mounts

```bash
sudo docker run --name edatest-check edatest:1.0
sudo docker inspect --format '{{.State.ExitCode}}' edatest-check
sudo docker inspect --format '{{json .Mounts}}' edatest-check
```

Expected: START policy=hfsm backend=verilator, a coverage curve, PASS, exit code 0, and Mounts []. The run performs 20000 cycles. Compilation may occur if required by the supplied Verilator adapter. If Mounts is not [], inspect inherited image volumes before claiming a completely mount-free run. A failure must be investigated; do not interpret a successful image build or Python smoke test as a successful RTL run.

Container names must be unused. If edatest-check already exists, use another name consistently in subsequent commands; do not delete a container with unsaved results.

The container is retained after exit. Save logs and results without a bind mount:

```bash
mkdir -p results
sudo docker logs edatest-check > results/run.log 2>&1
sudo docker cp edatest-check:/workspace/spi_xfer_public/spi_xfer_public/edatest_result.json results/
```

## Export image

Keep the exported archive outside this build directory:

```bash
sudo docker save edatest:1.0 -o ../edatest-1.0.tar
sudo chown "$USER":"$(id -gn)" ../edatest-1.0.tar
sha256sum ../edatest-1.0.tar > ../edatest-1.0.tar.sha256
ls -lh ../edatest-1.0.tar
```

Use docker save, not docker export: save preserves image metadata and its entry point.

## Recipient verification

On an independent Docker installation, place the tar and checksum together and run:

```bash
sha256sum -c edatest-1.0.tar.sha256
sudo docker load -i edatest-1.0.tar
sudo docker run --name edatest-reload-check edatest:1.0
```

The recipient does not need to load the base image separately because docker save includes the image's parent layers. Verify PASS, exit code 0, and mounts as above. Reloading on the original machine is useful but is not as strong a portability check as testing on another Docker installation. This package uses the existing x86_64 base image; other CPU architectures have not been tested.

## GitHub delivery

Commit these build files and a short validation record to the team's repository. Keep exported image archives out of normal Git history. Upload the tar and checksum as GitHub Release assets if each is under 2 GiB. GitHub blocks normal Git files larger than 100 MiB. If the tar is too large for a Release asset, check compressed size and agree on a distribution method before uploading. Do not blindly commit or publish credentials. The repository URL and visibility must be confirmed before publication.

## Validation status at preparation time

- The supplied inference_interface.py passed its own hfsm/random/greedy smoke tests on the preparation host.
- All included Python sources were syntax-checked.
- Docker image build, real RTL simulation, archive reload, and GitHub upload have NOT been performed on the preparation host. Complete the Ubuntu checks above before describing the image as validated.
