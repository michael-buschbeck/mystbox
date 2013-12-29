#!/usr/bin/perl

use strict;
use warnings;


sub parse_and_interpolate
{
  my $num_section = shift;
  my $label       = shift;
  my $mode        = shift;
  my $definition  = shift;

  my @point;

  while ($definition =~ /[ML] \s* ([0-9]+\.[0-9]+) \s+ ([0-9]+\.[0-9]+)/xg) {
    my $x = $1;
    my $y = $2;

    push @point, { x => $x, y => $y };
  }

  print "$label:\n";

  my $x_curve_from = $point[0]{x};
  my $y_curve_from = $point[0]{y};

  my $x_curve_to   = $point[-1]{x};
  my $y_curve_to;

  if ($mode eq 'continue') {
    $y_curve_to = $point[-1]{y};
  }
  elsif ($mode eq 'return') {
    foreach my $point (@point) {
      $y_curve_to = $point->{y}
        if not defined $y_curve_to or $y_curve_to > $point->{y};
    }
  }
  else {
    die "Invalid mode: $mode\n";
  }

  printf "                   %8.3f   %8.3f\n", $x_curve_from, $y_curve_from;

  SECTION:
  for (my $ind_section = 0; $ind_section < $num_section; ++$ind_section) {
    my $x = $x_curve_from + ($x_curve_to - $x_curve_from) * $ind_section / ($num_section - 1);

    for (my $ind_point = 0; $ind_point < @point - 1; ++$ind_point) {
      my ($x_point_from, $y_point_from) = ($point[$ind_point    ]{x}, $point[$ind_point    ]{y});
      my ($x_point_to,   $y_point_to  ) = ($point[$ind_point + 1]{x}, $point[$ind_point + 1]{y});

      if ($x_point_from <= $x && $x <= $x_point_to) {
        my $y = $y_point_from + ($x - $x_point_from) / ($x_point_to - $x_point_from) * ($y_point_to - $y_point_from);

        my $alpha_x = $ind_section / ($num_section - 1);
        my $alpha_y = ($y - $y_curve_from) / ($y_curve_to - $y_curve_from);

        $alpha_x = 0.0 if $alpha_x == -0.0;
        $alpha_y = 0.0 if $alpha_y == -0.0;

        printf "%4d   %4d/%-4d   %8.3f   %8.3f    %8.6f    %8.6f    %5d\n", $ind_section, $ind_point, @point - 2, $x, $y, $alpha_x, $alpha_y, 32768 * $alpha_y;
      }
    }
  }

  printf "                   %8.3f   %8.3f\n", $x_curve_to,   $y_curve_to;
}


parse_and_interpolate(64, 'regular', 'continue', q(
  M460.62991 396.84906L476.633616814 396.221887795L491.49543748 394.323069863L505.319179836 391.126654226L518.208651719 386.606688906L530.267660964 380.737221926L541.60001541 373.492301309L552.309522893 364.845975076L562.49999125 354.77229125L572.275228318 343.245297854L581.739041934 330.23904291L590.995239934 315.727574441L600.147630156 299.684940469L618.556218613 262.902368105L637.79527 219.68371L657.034318086 176.465051967L675.442904063 139.682479172L684.595293315 123.639844843L693.851490508 109.128375947L703.315303462 96.1221205276L713.09054 84.595126625L723.281007944 74.521442281L733.990515117 65.8751155371L745.322869341 58.6301944348L757.381878438 52.7607270156L770.271350229 48.240761321L784.095092539 45.0443453926L798.956913188 43.1455272717L814.96062 42.518355
));

parse_and_interpolate(64, 'overswing', 'continue', q(
  M460.62991 822.0459L476.445432639 821.490579812L490.75597377 819.788157402L503.68525071 816.883940002L515.356980781 812.723234844L525.894881301 807.251349158L535.42266959 800.413590176L544.064062966 792.155265129L551.94277875 782.42168125L559.18253426 771.158145769L565.907046816 758.309965918L572.240033738 743.822448928L578.305212344 727.640902031L590.127013887 689.976947441L602.36219 644.88056L621.294143291 575.740250586L638.383507422 520.276615156L653.863850029 477.218008555L661.073628148 459.943201953L667.96873875 445.292785625L674.578377789 433.107803926L680.931741221 423.229301211L687.058024999 415.498321836L692.986425078 409.755910156L698.746137413 405.843110527L704.366357959 403.600967305L709.87628267 402.870524844L715.3051075 403.4928275L720.682028405 405.308919629L726.036241338 408.159845586L736.793327109 416.330376406L759.319625 438.095395625L771.555972393 449.146593711L784.752542266 458.614723906L791.783900519 462.357749434L799.142902256 465.228141055L806.858743431 467.066943125L814.96062 467.7152
));

parse_and_interpolate(64, 'twitch', 'return', q(
  M460.62991 1247.2427L471.797902 1246.2262846L482.083527483 1243.26347L491.538690362 1238.48401604L500.215294551 1232.01768262L508.165243961 1223.99422959L515.440442507 1214.54341682L522.092794101 1203.79500419L528.174202656 1191.87875156L533.736572085 1178.92441881L538.831806301 1165.0617658L547.828484746 1135.13053848L555.579469294 1103.12314856L562.49999125 1070.077675L575.510572598 1005.02479277L582.431094597 975.093542017L590.182079219 948.276523438L594.498754795 936.360250888L599.178757766 925.611815991L604.273992044 916.160978616L609.836361543 908.137498633L615.917770175 901.67113591L622.570121853 896.891650317L629.84532049 893.928801724L637.79527 892.91235L645.745218601 893.928801736L653.020416387 896.891650415L659.672767268 901.67113624L665.754175156 908.137499414L671.316543962 916.160980142L676.411777598 925.611818628L681.091779973 936.360255075L685.408455 948.276529688L693.159438652 975.093554224L700.079959844 1005.02481387L713.09054 1070.077725L720.011061543 1103.12321975L727.762045781 1135.13063613L736.758724004 1165.06189578L741.853958137 1178.92456733L747.4163275 1191.87892031L753.497736003 1203.79519492L760.150087559 1214.54363137L767.425286077 1223.99446986L775.375235469 1232.01795059L784.051839646 1238.48431376L793.50700252 1243.26379958L803.792628 1246.22664826L814.96062 1247.2431
));
