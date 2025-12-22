<?php
// ===== version.php =====

header("Content-Type: application/octet-stream");

// EDIT THESE
$major = 2;
$minor = 6;

// Framed response
echo "<<";
echo $major . "." . $minor;
echo ">>";
exit;
