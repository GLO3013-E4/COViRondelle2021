FROM jjanzic/docker-python3-opencv:latest

RUN apt-get update && apt-get install -y --no-install-recommends tesseract-ocr

RUN mkdir -p /home/app/scripts
WORKDIR /home/app/scripts

COPY . .
COPY .pylintrc .
COPY requirements.txt .

RUN pip install -r requirements.txt

CMD ["bash"]
