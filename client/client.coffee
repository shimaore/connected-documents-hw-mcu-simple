fs = require 'fs'

device = '/dev/rfcomm0'
flags = 'w+'

fs.open device, flags, (err,fd) ->
  if err then throw err

  input = fs.createReadStream device, {flags,fd}
  output = fs.createWriteStream device, {flags,fd}

  input.on 'data', (chunk) ->
    console.log chunk

  input.on 'end', ->
    console.log 'Input stream END'

  input.on 'error', (error) ->
    console.log 'Input stream error: ' + error

  output.on 'error', (error) ->
    console.log 'Output stream error: ' + error

  setInterval ->
    output.write 'v32000\r\n'
    console.log 'Sent'
  , 1341

  setInterval ->
    output.write 'v64000\r\n'
    console.log 'Sent'
  , 697

  setInterval ->
    output.write 'v0\r\n'
    console.log 'Sent'
  , 2312

  setInterval ->
    output.write 'OK\n'
    console.log 'OK Sent'
  , 2000
