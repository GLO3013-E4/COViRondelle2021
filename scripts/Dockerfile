FROM jjanzic/docker-python3-opencv:latest

RUN apt-get update
RUN apt-get install -y poppler-utils
RUN apt-get install -y tesseract-ocr
RUN apt-get install -y libtesseract-dev
RUN apt-get install -y libleptonica-dev && ldconfig
RUN apt-get install -y libsm6 libxext6

RUN tesseract --version

RUN mkdir -p /home/app/scripts
WORKDIR /home/app/scripts

COPY . .

RUN pip install -r requirements.txt

CMD ["bash"]
