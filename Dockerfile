# builder image
FROM registry.fiblab.net/general/dev:latest as builder
RUN mkdir /build
COPY . /build
WORKDIR /build
RUN CGO_ENABLED=0 GOOS=linux go build -a -o routing .

# generate clean, final image for end users
FROM alpine:latest
COPY --from=builder /build/routing .

# executable
ENTRYPOINT [ "./routing" ]
