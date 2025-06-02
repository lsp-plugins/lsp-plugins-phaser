<?php
	plugin_header();
	$m      =   ($PAGE == 'phaser_mono') ? 'm' : 's';
?>

<p>
	This plugin allows to simulate the chorus phaser effect.
</p>
<p>
	The effect can be reached by applying multiple all-pass filters with varying frequency in time which is
	controlled by the low-frequency-oscillator (LFO). The processed signal mixed with the unprocessed signal
	gives series of notches in the frequency domain which apply the corresponding character to the sound.
	Additionally, more deep effect can be reached by passing the processed output to the input of the plugin
	by using the feedback delay line. 
</p>

<p><b>'LFO' section:</b></p>
<ul>
	<li><b>Restart</b> - resets the current phase of the LFO to the initial value.</li>
	<li><b>Filters</b> - the number of all-pass filters used.</li>
	<li>
		<b>Type</b> - allows to configure the type of the oscillation form <?= ($m == 'm') ? '' : 'for left and right channels respectively' ?>.
		The oscillator can work in three modes: full period, the first half of the period (raising) and the second half of the period (falling).
		When using half period mode, then additional crossfade is performed to make the smooth transform of the sound.
	</li>
	<li><b>Rate/Tempo</b> - the combo box that allows to select how to set the oscillation rate: using rate knob or tempo settings.</li>
	<li><b>Rate knob</b> - the knob that allows to control the oscillation rate.</li>
    <li><b>Tempo</b> - the knob that allows to control the tempo which can be used for computing oscillation rate.</li>
    <li><b>Sync</b> - enables tempo synchronization with host or DAW.</li>
    <li><b>Tap</b> - the button that allows to estimate a tempo by performing series of clicks on it.</li>
    <li><b>Crossfade</b> - the part of the LFO period to perform the crossfade. Allows to avoid popping sound when LFO is used in half-period mode.</li>
	<li><b>Overlap</b> - the overlap amount between paths of all-pass filters.</li>
    <li><b>Phase</b> - the initial phase of the LFO, used by the <b>Restart</b> button.</li>
    <?php if ($m == 's') { ?>
    <li><b>Difference</b> the phase difference between left and right (or mid and side) channels.</li>
    <?php } ?>
	<li><b>Phase range</b> - the phase range used to evenly assign the phase for each all-pass filter.</li>
</ul>

<p><b>'Controls' section:</b></p>
<ul>
	<?php if ($m == 's') { ?>
	<li><b>Mid/Side</b> - switches the effect to work on Mid/Side components of the signal instead of Left and Right.</li>
	<li><b>Mono</b> - allows to test the output of the plugin for mono compatibility.</li>
	<?php } ?>
	<li><b>Phase</b> - the button that allows to enable phase inversion of the processed signal which is added to original one.</li>
	<li><b>Frequency</b> - set of two knobs that allow to set the frequency range for all-pass filters.</li>
	<li><b>Frequency Link</b> - button that allows to link frequency buttons together to keep constant logarithmic frequency range.</li>
	<li><b>Quality</b> - the quality factor of all-pass filters.</li>
	<li><b>Depth</b> - the amount of processed signal added to the unprocessed signal.</li>
	<li><b>Feedback</b> - the button that switches on the feedback chain.</li>
    <li><b>Feedback Phase</b> - the button that allows to invert the polarity of the feedback signal.</li>
	<li><b>Feedback</b> - the knob that controls the amount of signal being fed back.</li>
    <li><b>Feedback Delay</b> - the additional delay which can be applied to the feedback signal.</li>
</ul>

<p><b>'Signal' section:</b></p>
<ul>
	<li><b>Input</b> - the amount of input signal.</li>
	<li><b>Dry</b> - the amount of dry (unprocessed) signal.</li>
	<li><b>Wet</b> - the amount of wet (processed) signal.</li>
	<li><b>Dry/Wet</b> - the balance between dry (unprocessed) signal and mixed signal formed by Dry/Wet knobs.</li>
	<li><b>Output</b> - the output volume of the plugin.</li>
</ul>
