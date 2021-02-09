FROM jjanzic/docker-python3-opencv:latest

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        poppler-utils
        tesseract-ocr
        libtesseract-dev
        libleptonica-dev
    && ldconfig \
    && apt-get install -y --no-install-recommends \
        libsm6 \
        libxext6
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN tesseract --version

RUN mkdir -p /home/app/scripts
WORKDIR /home/app/scripts

COPY . .

RUN pip install -r requirements.txt

CMD ["bash"]
