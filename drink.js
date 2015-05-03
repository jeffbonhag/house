var process = require('process');

var fs = require('fs');
var bytes = [];
var maxSampleLength = 10000;

var buffer = fs.readFileSync(process.argv[2]);

for (var i=0; i < buffer.length && i < maxSampleLength; i++) {
	bytes.push(buffer[i]);
}

var lengthPart = "const int sounddata_length = " + bytes.length + ";\r\n"
var otherPart = "const unsigned char sounddata_data[] PROGMEM = {" + bytes.join(",") + "};\r\n";

fs.writeFileSync(process.argv[2] + '.h', lengthPart + otherPart);