fs = require 'fs'

device = '/dev/rfcomm0'
flags = 'w+'

zappa = require 'zappajs'

console.log "Opening #{device} with flags #{flags}"
fs.open device, flags, (err,fd) ->
  if err then throw err

  input = fs.createReadStream device, {flags,fd}
  output = fs.createWriteStream device, {flags,fd}

  input.on 'data', (chunk) ->
    console.log chunk

  input.on 'end', ->
    console.log 'Input stream END'
    throw 'Done'

  input.on 'error', (error) ->
    console.error 'Input stream'
    throw error

  output.on 'error', (error) ->
    console.error 'Output stream'
    throw error

  output.write '\r\n!s\r\n'

  s = zappa ->

    @put '/value/:value', ->
      output.write "\r\n!v#{@params.value}\r\n"
      @json ok:true
