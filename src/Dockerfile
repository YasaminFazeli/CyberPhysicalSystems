##################################################
# Section 1: Build the application
FROM alpine:latest as builder
MAINTAINER Christian Berger christian.berger@gu.se

ADD . /opt/sources
WORKDIR /opt/sources

# In the block below we first get all the dependencies
# We then remove all the caches
# Then, we build and copy the application and lastly delete the source folder and dependencies
RUN apk --no-cache add cmake make g++ && \
    rm -fr /var/cache/apk/* && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release .. && \
    make && make test && \
    strip helloworld && \
    cp helloworld / && \
    cd ../.. && \
    rm -fr sources/ && \
    apk del --purge cmake make g++

##################################################
# Section 2: Bundle the application.
FROM scratch
MAINTAINER Christian Berger christian.berger@gu.se

# Here we copy the executable
COPY --from=builder /helloworld .
ENTRYPOINT ["/helloworld"]
