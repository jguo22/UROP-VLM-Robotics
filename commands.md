ssh -v -N -L 8778:node0000:8777 orcd-login
curl -sv http://localhost:8778/docs

export SSL_CERT_FILE="$(python -c 'import certifi; print(certifi.where())')"