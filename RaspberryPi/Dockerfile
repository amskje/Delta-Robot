FROM arm64v8/ros:humble

#Install pip and system packages
RUN apt update && apt install -y \
	python3-pip \
	python3-tk \
	&& rm -rf /var/lib/apt/lists/*

#Install Python libaries
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

#Set working directory inside the container
WORKDIR /home/dev/Delta-Robot




#Default command
CMD ["bash"]

#This is how to run the python script direclty
#CMD ["python3", "gui_script.py"]
