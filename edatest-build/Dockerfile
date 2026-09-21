FROM eda-coverage-base:1.0

WORKDIR /workspace/spi_xfer_public/spi_xfer_public
COPY inference_interface.py harness.py run_agent.py ./

ENTRYPOINT ["python3", "-u", "run_agent.py"]
CMD ["--steps", "20000"]
