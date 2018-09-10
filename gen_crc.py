import crc16
appfile = open('application.bin', 'rb')
byte_seq = appfile.read()
crc = crc16.crc16xmodem(byte_seq, 0xffff)
print '{:04X}'.format(crc & 0xffff)
print 'Remember endianness -> Byte swapping...'
print 'Add CRC at last two bytes of .dat'


