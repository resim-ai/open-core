FROM ubuntu:latest as production

LABEL description="Core ReSim Run container"

COPY resim_run /resim/
COPY run_sim.sh /resim/

WORKDIR /resim/
ENTRYPOINT ["/resim/run_sim.sh"]
