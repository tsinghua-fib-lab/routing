# builder image
FROM git.tsingroc.com:5050/general/dev:latest as builder
RUN mkdir /build
COPY . /build
WORKDIR /build
RUN ./init_module.sh && CGO_ENABLED=0 GOOS=linux go build -a -o routing .

# generate clean, final image for end users
FROM alpine:latest
COPY --from=builder /build/routing .

# executable
ENTRYPOINT [ "./routing" ]
