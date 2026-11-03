ARG BUILD_FROM
FROM ${BUILD_FROM}

# numpy from Alpine packages: prebuilt for every arch, no compiler needed.
RUN apk add --no-cache python3 py3-numpy \
    && python3 -m venv --system-site-packages /opt/venv

COPY app /opt/app
RUN /opt/venv/bin/pip install --no-cache-dir /opt/app

ENV NEATO_DATA_DIR=/data \
    NEATO_PORT=8099 \
    PYTHONUNBUFFERED=1

CMD ["/opt/venv/bin/python", "-m", "neato_rooms"]

LABEL io.hass.type="addon" io.hass.version="0.1.0"
