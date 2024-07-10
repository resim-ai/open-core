FROM ubuntu

RUN apt update
RUN apt -y install wget python3 python3-pip git

COPY ../../../requirements.txt /
RUN pip3 install -r requirements.txt

COPY src/ /workspaces/

WORKDIR /usr/bin

ENTRYPOINT /workspaces/entrypoint.py
