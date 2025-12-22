<?php
// ===== firmware.php =====
header("Content-Type: application/octet-stream");

$firmware_path = __DIR__ . "/firmware.bin";
$block_size = 1024;

// Frame bytes
$SOF = 0xAA;
$EOF = 0x55;

// Validate block index
if (!isset($_GET['i'])) {
    http_response_code(400);
    exit;
}

$i = intval($_GET['i']);
if ($i < 0) {
    http_response_code(400);
    exit;
}

// Open firmware
$fh = fopen($firmware_path, "rb");
if (!$fh) {
    http_response_code(500);
    exit;
}

// Seek to block
$offset = $i * $block_size;
fseek($fh, $offset);

// Read block
$data = fread($fh, $block_size);
fclose($fh);

$len = strlen($data);

// ---- FRAME START ----
echo chr($SOF);

// EOF block
if ($len === 0) {
    echo pack("v", 0);   // length = 0
    echo chr($EOF);
    exit;
}

// Compute XOR checksum
$checksum = 0;
for ($j = 0; $j < $len; $j++) {
    $checksum ^= ord($data[$j]);
}

// Payload
echo pack("v", $len);       // length (LE)
echo $data;                 // firmware bytes
echo pack("C", $checksum);  // checksum

// ---- FRAME END ----
echo chr($EOF);
exit;
