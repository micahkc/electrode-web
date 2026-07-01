<script lang="ts">
  import { onMount } from 'svelte';
  import IndoorScene from '$lib/components/IndoorScene.svelte';
  import {
    Activity,
    AlertTriangle,
    BatteryCharging,
    CirclePause,
    CirclePlay,
    Command,
    Download,
    Gauge,
    MapPinned,
    Plug,
    Power,
    Radio,
    RotateCcw,
    Satellite,
    ShieldCheck,
    Square,
    Upload,
    Wifi
  } from '@lucide/svelte';
  import {
    COMMAND_DEFINITIONS,
    TOPIC_DEFINITIONS,
    createPlotPacketCatalog,
    createInitialVehicleState,
    plotPacketKey,
    type CommandName,
    type CommandResult,
    type ConnectionState,
    type PlotFieldDefinition,
    type PlotPacketDefinition,
    type PlotSeries,
    type PlotState,
    type ReplayState,
    type RuntimeMode,
    type SynapseLogExport,
    type VehicleState
  } from '@electrode/sdk';

  type WorkerOut =
    | { type: 'state'; state: VehicleState }
    | { type: 'connection'; connection: ConnectionState }
    | { type: 'commandResult'; result: CommandResult }
    | { type: 'recording'; recording: boolean; count: number }
    | { type: 'recordingExport'; export: SynapseLogExport }
    | { type: 'plotState'; plotState: PlotState }
    | { type: 'replay'; replay: ReplayState };

  type MapViewMode = '2d' | '3d';
  type PlotTraceSelection = {
    packetKey: string;
    fieldPath: string;
  };

  type ResolvedPlotTrace = PlotTraceSelection & {
    color: string;
    packet: PlotPacketDefinition | undefined;
    field: PlotFieldDefinition | undefined;
    series: PlotSeries | undefined;
    path: string;
    lastValue: number | null;
    rangeLabel: string;
  };

  const vehicleId = 'electrode-01';
  const runtimeModes: RuntimeMode[] = ['simulator', 'zenoh', 'bridge', 'replay'];
  const mapViewModes: MapViewMode[] = ['2d', '3d'];
  const plotTraceColors = ['#42e8c4', '#ffbf57'] as const;
  const commandEntries = Object.entries(COMMAND_DEFINITIONS) as Array<[CommandName, (typeof COMMAND_DEFINITIONS)[CommandName]]>;
  const flightModes = ['hold', 'manual', 'mission', 'return', 'land'];
  const manualChannels = [
    ['THR', '0'],
    ['ROLL', '1'],
    ['PITCH', '2'],
    ['YAW', '3'],
    ['MODE', '4']
  ] as const;
  const manualLinks = [
    ['Manual', 'synapse/manual_control'],
    ['Auto', 'synapse/control_output'],
    ['Serial', '/dev/ttyACM0 · 57600']
  ] as const;
  const rollTicks = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];
  const pitchMarks = [-30, -20, -10, 10, 20, 30];

  let worker: Worker | null = null;
  let runtimeMode: RuntimeMode = 'simulator';
  let mapViewMode: MapViewMode = initialMapViewMode();
  let zenohEndpoint = 'ws/127.0.0.1:7447';
  let bridgeUrl = 'ws://127.0.0.1:8787/ws';
  let selectedMode = 'hold';
  let vehicle = createInitialVehicleState(vehicleId);
  let connection: ConnectionState = { mode: 'simulator', status: 'disconnected', url: '', message: 'offline' };
  let replay: ReplayState = { loaded: false, playing: false, cursorMs: 0, durationMs: 0, speed: 1, frameCount: 0 };
  let recording = false;
  let recordingCount = 0;
  let plotState: PlotState = { packets: createPlotPacketCatalog(vehicleId), series: [], updatedAtMs: 0 };
  let plotTraces: PlotTraceSelection[] = [
    { packetKey: plotPacketKey(`vehicle/${vehicleId}/state/pose`, 'Pose'), fieldPath: 'altM' },
    { packetKey: plotPacketKey(`vehicle/${vehicleId}/state/velocity`, 'Velocity'), fieldPath: 'groundSpeedMps' }
  ];

  $: topicRows = Object.values(vehicle.topics).sort((a, b) => a.label.localeCompare(b.label));
  $: warningCount = vehicle.events.filter((event) => event.severity !== 'info').length;
  $: pose = vehicle.pose;
  $: attitude = vehicle.attitude;
  $: battery = vehicle.battery;
  $: link = vehicle.link;
  $: mapSubtitle =
    mapViewMode === '2d'
      ? pose
        ? `${pose.lat.toFixed(6)}, ${pose.lon.toFixed(6)}`
        : 'no fix'
      : `${vehicle.localization.source} · local ${format(pose?.xM)} / ${format(pose?.yM)} m`;
  $: mapX = clamp(50 + (pose?.xM ?? 0) * 0.72, 8, 92);
  $: mapY = clamp(50 - (pose?.yM ?? 0) * 0.72, 8, 92);
  $: yawDeg = attitude?.yawDeg ?? 0;
  $: rollDeg = attitude?.rollDeg ?? 0;
  $: pitchDeg = attitude?.pitchDeg ?? 0;
  $: bankPointerDeg = clamp(rollDeg, -60, 60);
  $: pitchOffset = clamp(pitchDeg * 2.2, -54, 54);
  $: staleTopics = topicRows.filter((topic) => topic.stale).length;
  $: plotPackets = plotState.packets;
  $: {
    const nextPlotTraces = normalizePlotTraces(plotTraces, plotPackets);
    if (!samePlotTraces(nextPlotTraces, plotTraces)) {
      plotTraces = nextPlotTraces;
    }
  }
  $: resolvedPlotTraces = plotTraces.map((trace, index) =>
    resolvePlotTrace(trace, plotTraceColors[index] ?? '#42e8c4', plotState, plotPackets)
  );
  $: plotHasSamples = resolvedPlotTraces.some((trace) => trace.series && trace.series.samples.length > 1);

  onMount(() => {
    worker = new Worker(new URL('$lib/workers/comms.worker.ts', import.meta.url), { type: 'module' });
    worker.addEventListener('message', (event: MessageEvent<WorkerOut>) => {
      const message = event.data;

      if (message.type === 'state') {
        vehicle = message.state;
      } else if (message.type === 'connection') {
        connection = message.connection;
      } else if (message.type === 'recording') {
        recording = message.recording;
        recordingCount = message.count;
      } else if (message.type === 'recordingExport') {
        exportRecordingFile(message.export);
      } else if (message.type === 'plotState') {
        plotState = message.plotState;
      } else if (message.type === 'replay') {
        replay = message.replay;
      }
    });

    connect();

    return () => {
      worker?.postMessage({ type: 'disconnect' });
      worker?.terminate();
      worker = null;
    };
  });

  function connect(): void {
    const url = runtimeMode === 'zenoh' ? zenohEndpoint : bridgeUrl;
    worker?.postMessage({ type: 'connect', mode: runtimeMode, url, vehicleId });
  }

  function initialMapViewMode(): MapViewMode {
    if (typeof window === 'undefined') {
      return '3d';
    }

    const requested = new URLSearchParams(window.location.search).get('map') as MapViewMode | null;
    return requested && mapViewModes.includes(requested) ? requested : '3d';
  }

  function disconnect(): void {
    worker?.postMessage({ type: 'disconnect' });
  }

  function sendCommand(command: CommandName): void {
    const definition = COMMAND_DEFINITIONS[command];
    if (definition.requiresConfirmation && !window.confirm(`${definition.description} for ${vehicle.vehicleId}?`)) {
      return;
    }

    const args = command === 'setMode' ? { mode: selectedMode } : {};
    worker?.postMessage({ type: 'command', command, args });
  }

  function startRecording(): void {
    worker?.postMessage({ type: 'startRecording' });
  }

  function stopRecording(): void {
    worker?.postMessage({ type: 'stopRecording' });
  }

  function exportRecording(): void {
    worker?.postMessage({ type: 'exportRecording' });
  }

  function loadReplay(event: Event): void {
    const input = event.currentTarget as HTMLInputElement;
    const file = input.files?.[0];

    if (!file) {
      worker?.postMessage({ type: 'loadReplay' });
      return;
    }

    const reader = new FileReader();
    reader.addEventListener('load', () => {
      if (!(reader.result instanceof ArrayBuffer)) {
        return;
      }
      const bytes = new Uint8Array(reader.result);
      runtimeMode = 'replay';
      worker?.postMessage({ type: 'loadReplay', bytes }, [bytes.buffer]);
    });
    reader.readAsArrayBuffer(file);
  }

  function playReplay(): void {
    worker?.postMessage({ type: 'playReplay' });
  }

  function pauseReplay(): void {
    worker?.postMessage({ type: 'pauseReplay' });
  }

  function seekReplay(event: Event): void {
    const input = event.currentTarget as HTMLInputElement;
    worker?.postMessage({ type: 'seekReplay', cursorMs: Number(input.value) });
  }

  function setReplaySpeed(event: Event): void {
    const input = event.currentTarget as HTMLInputElement;
    worker?.postMessage({ type: 'setReplaySpeed', speed: Number(input.value) });
  }

  function exportRecordingFile(log: SynapseLogExport): void {
    const buffer = new ArrayBuffer(log.bytes.byteLength);
    new Uint8Array(buffer).set(log.bytes);
    const blob = new Blob([buffer], { type: log.mimeType });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = log.filename;
    anchor.click();
    URL.revokeObjectURL(url);
  }

  function plotSeriesPath(samples: PlotSeries['samples'], width: number, height: number): string {
    if (samples.length < 2) {
      return '';
    }

    const values = samples.map((sample) => sample.value);
    const min = Math.min(...values);
    const max = Math.max(...values);
    const range = Math.max(0.001, max - min);
    const startTime = samples[0].timeMs;
    const timeSpan = Math.max(1, samples.at(-1)!.timeMs - startTime);

    return samples
      .map((sample, index) => {
        const x = ((sample.timeMs - startTime) / timeSpan) * width;
        const y = height - ((sample.value - min) / range) * height;
        return `${index === 0 ? 'M' : 'L'} ${x.toFixed(1)} ${y.toFixed(1)}`;
      })
      .join(' ');
  }

  function normalizePlotTraces(traces: PlotTraceSelection[], packets: PlotPacketDefinition[]): PlotTraceSelection[] {
    return traces.map((trace, index) => {
      const packet = packets.find((candidate) => candidate.key === trace.packetKey) ?? defaultPlotPacket(packets, index);
      const field = packet?.fields.find((candidate) => candidate.path === trace.fieldPath) ?? packet?.fields[0];
      return {
        packetKey: packet?.key ?? trace.packetKey,
        fieldPath: field?.path ?? trace.fieldPath
      };
    });
  }

  function samePlotTraces(left: PlotTraceSelection[], right: PlotTraceSelection[]): boolean {
    return left.length === right.length && left.every((trace, index) => trace.packetKey === right[index].packetKey && trace.fieldPath === right[index].fieldPath);
  }

  function defaultPlotPacket(packets: PlotPacketDefinition[], index: number): PlotPacketDefinition | undefined {
    const preferredSchema = index === 0 ? 'Pose' : 'Velocity';
    return (
      packets.find((packet) => packet.active && packet.schema === preferredSchema) ??
      packets.find((packet) => packet.schema === preferredSchema) ??
      packets.find((packet) => packet.active) ??
      packets[0]
    );
  }

  function fieldsForPacket(packetKey: string): PlotFieldDefinition[] {
    return plotPackets.find((packet) => packet.key === packetKey)?.fields ?? [];
  }

  function updatePlotTrace(index: number, patch: Partial<PlotTraceSelection>): void {
    const next = [...plotTraces];
    const current = next[index];
    const packetKey = patch.packetKey ?? current.packetKey;
    next[index] = {
      packetKey,
      fieldPath: patch.packetKey && !patch.fieldPath ? fieldsForPacket(packetKey)[0]?.path ?? '' : patch.fieldPath ?? current.fieldPath
    };
    plotTraces = next;
  }

  function resolvePlotTrace(
    trace: PlotTraceSelection,
    color: string,
    state: PlotState,
    packets: PlotPacketDefinition[]
  ): ResolvedPlotTrace {
    const packet = packets.find((candidate) => candidate.key === trace.packetKey);
    const field = packet?.fields.find((candidate) => candidate.path === trace.fieldPath);
    const series = state.series.find((candidate) => candidate.packetKey === trace.packetKey && candidate.fieldPath === trace.fieldPath);
    const samples = series?.samples ?? [];
    return {
      ...trace,
      color,
      packet,
      field,
      series,
      path: plotSeriesPath(samples, 360, 132),
      lastValue: series?.lastValue ?? null,
      rangeLabel: formatPlotRange(samples, field?.units ?? '')
    };
  }

  function packetOptionLabel(packet: PlotPacketDefinition): string {
    const prefix = packet.source === 'synapse_fbs' ? 'Synapse' : packet.source === 'electrode' ? 'GCS' : 'Live';
    return `${prefix} · ${packet.label}${packet.active ? ' · live' : ''}`;
  }

  function formatPlotValue(value: number | null, units = ''): string {
    if (value === null || Number.isNaN(value)) {
      return '--';
    }

    const abs = Math.abs(value);
    const digits = abs >= 100 ? 0 : abs >= 10 ? 1 : 2;
    return `${value.toFixed(digits)}${units ? ` ${units}` : ''}`;
  }

  function formatPlotRange(samples: PlotSeries['samples'], units: string): string {
    if (samples.length < 2) {
      return 'no samples';
    }

    const values = samples.map((sample) => sample.value);
    return `${formatPlotValue(Math.min(...values), units)} - ${formatPlotValue(Math.max(...values), units)}`;
  }

  function clamp(value: number, min: number, max: number): number {
    return Math.min(max, Math.max(min, value));
  }

  function format(value: number | undefined, digits = 1): string {
    return value === undefined || Number.isNaN(value) ? '--' : value.toFixed(digits);
  }

  function formatDuration(ms: number): string {
    const seconds = Math.floor(ms / 1000);
    const minutes = Math.floor(seconds / 60);
    return `${minutes}:${String(seconds % 60).padStart(2, '0')}`;
  }
</script>

<svelte:head>
  <title>electrode GCS</title>
  <meta name="description" content="electrode web ground station" />
</svelte:head>

<main class="shell">
  <header class="topbar">
    <div class="brand">
      <div class="brand-mark">
        <img src="/cognipilot-mark.png" alt="CogniPilot" />
      </div>
      <div>
        <h1>electrode</h1>
        <p>{vehicle.vehicleId}</p>
      </div>
    </div>

    <div class="mode-control" aria-label="Runtime mode">
      {#each runtimeModes as option}
        <label class:active={runtimeMode === option}>
          <input type="radio" name="mode" value={option} bind:group={runtimeMode} />
          {option}
        </label>
      {/each}
    </div>

    <div class="header-actions">
      <button type="button" class="icon-button primary" title="Connect" onclick={connect}>
        <Plug size={18} />
        <span>Connect</span>
      </button>
      <button type="button" class="icon-button quiet" title="Disconnect" onclick={disconnect}>
        <Power size={18} />
      </button>
    </div>
  </header>

  <section class="status-strip">
    <div class="strip-item" class:ok={vehicle.connected} class:warn={!vehicle.connected}>
      <Wifi size={18} />
      <span>{connection.status}</span>
    </div>
    <div class="strip-item" class:ok={vehicle.localization.fresh} class:warn={!vehicle.localization.fresh}>
      <Satellite size={18} />
      <span>{vehicle.localization.source}</span>
    </div>
    <div class="strip-item" class:ok={!vehicle.mode.failsafe} class:danger={vehicle.mode.failsafe}>
      <ShieldCheck size={18} />
      <span>{vehicle.mode.failsafe ? 'failsafe' : 'nominal'}</span>
    </div>
    <div class="strip-item" class:warn={staleTopics > 0} class:ok={staleTopics === 0 && topicRows.length > 0}>
      <Activity size={18} />
      <span>{staleTopics} stale</span>
    </div>
  </section>

  <div class="dashboard">
    <section class="panel connection-panel">
      <div class="panel-heading">
        <div>
          <h2>Connection</h2>
          <p>{connection.message}</p>
        </div>
        <Radio size={20} />
      </div>

      {#if runtimeMode === 'zenoh'}
        <label class="field">
          <span>Zenoh endpoint or JSON5 config</span>
          <input bind:value={zenohEndpoint} spellcheck="false" />
        </label>
      {:else if runtimeMode === 'bridge'}
        <label class="field">
          <span>Bridge URL</span>
          <input bind:value={bridgeUrl} spellcheck="false" />
        </label>
      {:else}
        <div class="transport-readout">
          <span>Transport</span>
          <strong>{runtimeMode === 'replay' ? 'replay://memory' : 'simulator://electrode'}</strong>
        </div>
      {/if}

      <div class="button-row">
        <button type="button" class="icon-button primary" onclick={connect}>
          <Plug size={18} />
          <span>Open</span>
        </button>
        <button type="button" class="icon-button quiet" onclick={disconnect}>
          <Square size={18} />
          <span>Close</span>
        </button>
      </div>
    </section>

    <section class="panel vehicle-panel">
      <div class="panel-heading">
        <div>
          <h2>Vehicle</h2>
          <p>{vehicle.mode.name} · {vehicle.mode.armed ? 'armed' : 'disarmed'}</p>
        </div>
        <Gauge size={20} />
      </div>

      <div class="metrics">
        <div class="metric">
          <span>Altitude</span>
          <strong>{format(pose?.altM)} m</strong>
        </div>
        <div class="metric">
          <span>Speed</span>
          <strong>{format(vehicle.velocity?.groundSpeedMps)} m/s</strong>
        </div>
        <div class="metric">
          <span>Yaw</span>
          <strong>{format(attitude?.yawDeg, 0)} deg</strong>
        </div>
        <div class="metric">
          <span>Quality</span>
          <strong>{format(vehicle.localization.quality * 100, 0)}%</strong>
        </div>
      </div>
    </section>

    <section class="panel map-panel">
      <div class="panel-heading map-heading">
        <div>
          <h2>Map</h2>
          <p>{mapSubtitle}</p>
        </div>
        <div class="map-tools">
          <div class="map-view-control" aria-label="Map view mode">
            {#each mapViewModes as option}
              <button
                type="button"
                class:active={mapViewMode === option}
                onclick={() => {
                  mapViewMode = option;
                }}
              >
                {option.toUpperCase()}
              </button>
            {/each}
          </div>
          <MapPinned size={20} />
        </div>
      </div>

      {#if mapViewMode === '2d'}
        <div class="map-canvas outdoor-map">
          <svg class="mission-line" viewBox="0 0 100 100" aria-hidden="true">
            <path d="M18 72 C28 26 48 24 60 42 S78 74 88 28" />
          </svg>
          <div class="map-home">HOME</div>
          <div
            class="vehicle-marker"
            style={`left: ${mapX}%; top: ${mapY}%; transform: translate(-50%, -50%) rotate(${yawDeg}deg);`}
            title="Vehicle position"
          >
            <svg class="map-axes-marker" viewBox="-24 -24 48 48" aria-hidden="true">
              <defs>
                <marker id="map-axis-forward" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="5" markerHeight="5" orient="auto">
                  <path d="M0 0 L8 4 L0 8 Z" />
                </marker>
                <marker id="map-axis-right" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="4.5" markerHeight="4.5" orient="auto">
                  <path d="M0 0 L8 4 L0 8 Z" />
                </marker>
              </defs>
              <line class="map-axis-forward" x1="0" y1="0" x2="0" y2="-17" marker-end="url(#map-axis-forward)" />
              <line class="map-axis-right" x1="0" y1="0" x2="15" y2="0" marker-end="url(#map-axis-right)" />
              <circle cx="0" cy="0" r="3.2" />
            </svg>
          </div>
          <div class="map-scale">50 m</div>
        </div>
      {:else}
        <IndoorScene pose={pose} attitude={attitude} localizationQuality={vehicle.localization.quality} />
      {/if}
    </section>

    <section class="panel command-panel">
      <div class="panel-heading">
        <div>
          <h2>Commands</h2>
          <p>{vehicle.commandHistory[0]?.status ?? 'ready'}</p>
        </div>
        <Command size={20} />
      </div>

      <label class="field">
        <span>Mode</span>
        <select bind:value={selectedMode}>
          {#each flightModes as modeName}
            <option value={modeName}>{modeName}</option>
          {/each}
        </select>
      </label>

      <div class="command-grid">
        {#each commandEntries as [name, definition]}
          <button
            type="button"
            class:danger={name === 'land' || name === 'disarm'}
            class:primary={name === 'arm'}
            class="command-button"
            onclick={() => sendCommand(name)}
          >
            <span>{definition.label}</span>
          </button>
        {/each}
      </div>
    </section>

    <section class="panel manual-panel">
      <div class="panel-heading">
        <div>
          <h2>Manual Link</h2>
          <p>native · standby</p>
        </div>
        <Radio size={20} />
      </div>

      <div class="manual-service">
        {#each manualLinks as [label, value]}
          <div>
            <span>{label}</span>
            <strong>{value}</strong>
          </div>
        {/each}
      </div>

      <div class="channel-strip" aria-label="PPM channel map">
        {#each manualChannels as [label, channel]}
          <div>
            <span>{label}</span>
            <strong>{channel}</strong>
          </div>
        {/each}
      </div>
    </section>

    <section class="panel attitude-panel">
      <div class="panel-heading">
        <div>
          <h2>HUD</h2>
          <p>{vehicle.mode.name} · {vehicle.mode.armed ? 'armed' : 'disarmed'} · roll {format(attitude?.rollDeg)} · pitch {format(attitude?.pitchDeg)}</p>
        </div>
        <Activity size={20} />
      </div>

      <div class="metrics hud-metrics">
        <div class="metric">
          <span>Altitude</span>
          <strong>{format(pose?.altM)} m</strong>
        </div>
        <div class="metric">
          <span>Speed</span>
          <strong>{format(vehicle.velocity?.groundSpeedMps)} m/s</strong>
        </div>
        <div class="metric">
          <span>Yaw</span>
          <strong>{format(attitude?.yawDeg, 0)} deg</strong>
        </div>
        <div class="metric">
          <span>Quality</span>
          <strong>{format(vehicle.localization.quality * 100, 0)}%</strong>
        </div>
      </div>

      <div class="attitude-display">
        <svg class="attitude-instrument" viewBox="0 0 260 260" role="img" aria-label="Attitude indicator">
          <defs>
            <clipPath id="attitude-face">
              <circle cx="130" cy="130" r="90" />
            </clipPath>
            <radialGradient id="attitude-glass" cx="36%" cy="18%" r="78%">
              <stop offset="0%" stop-color="#ffffff" stop-opacity="0.12" />
              <stop offset="45%" stop-color="#ffffff" stop-opacity="0.03" />
              <stop offset="100%" stop-color="#000000" stop-opacity="0.18" />
            </radialGradient>
          </defs>

          <rect class="instrument-plate" x="24" y="24" width="212" height="212" rx="18" />
          <circle class="mount-screw" cx="51" cy="51" r="4.8" />
          <circle class="mount-screw" cx="209" cy="51" r="4.8" />
          <circle class="mount-screw" cx="51" cy="209" r="4.8" />
          <circle class="mount-screw" cx="209" cy="209" r="4.8" />
          <circle class="bezel-outer" cx="130" cy="130" r="112" />
          <circle class="bezel-inner" cx="130" cy="130" r="96" />

          <g clip-path="url(#attitude-face)" transform={`rotate(${rollDeg} 130 130)`}>
            <g transform={`translate(0 ${pitchOffset})`}>
              <rect class="sky" x="0" y="-80" width="260" height="210" />
              <rect class="ground" x="0" y="130" width="260" height="230" />
              <line class="horizon-line" x1="35" y1="130" x2="225" y2="130" />
              {#each pitchMarks as mark}
                <g class="pitch-mark" transform={`translate(0 ${-mark * 3})`}>
                  <line x1={Math.abs(mark) % 20 === 0 ? 84 : 99} y1="130" x2={Math.abs(mark) % 20 === 0 ? 176 : 161} y2="130" />
                  <text x={Math.abs(mark) % 20 === 0 ? 71 : 88} y="134">{Math.abs(mark)}</text>
                  <text x={Math.abs(mark) % 20 === 0 ? 183 : 166} y="134">{Math.abs(mark)}</text>
                </g>
              {/each}
            </g>
          </g>

          <circle class="face-shadow" cx="130" cy="130" r="91" />
          <g class="roll-scale">
            {#each rollTicks as tick}
              <line class:major={Math.abs(tick) === 30 || tick === 0} x1="130" y1="28" x2="130" y2={Math.abs(tick) === 30 || tick === 0 ? 48 : 40} transform={`rotate(${tick} 130 130)`} />
            {/each}
          </g>
          <path class="top-index" d="M130 42 L121 60 H139 Z" />
          <g class="bank-pointer" transform={`rotate(${bankPointerDeg} 130 130)`}>
            <path d="M130 61 L121 77 H139 Z" />
            <line x1="130" y1="54" x2="130" y2="65" />
          </g>
          <g class="aircraft-reference">
            <path d="M88 133 H116 L124 124 L130 145 L136 124 L144 133 H172" />
            <circle cx="130" cy="133" r="3.5" />
          </g>
          <circle class="glass" cx="130" cy="130" r="91" />
        </svg>
      </div>
    </section>

    <section class="panel power-panel">
      <div class="panel-heading">
        <div>
          <h2>Power Link</h2>
          <p>{format(link?.latencyMs, 0)} ms · {format(link?.rssiDbm, 0)} dBm</p>
        </div>
        <BatteryCharging size={20} />
      </div>

      <div class="battery-bar" aria-label="Battery remaining">
        <span style={`width: ${clamp(battery?.remainingPct ?? 0, 0, 100)}%;`}></span>
      </div>
      <div class="metrics compact">
        <div class="metric">
          <span>Voltage</span>
          <strong>{format(battery?.voltageV, 2)} V</strong>
        </div>
        <div class="metric">
          <span>Current</span>
          <strong>{format(battery?.currentA, 1)} A</strong>
        </div>
      </div>
    </section>

    <section class="panel replay-panel">
      <div class="panel-heading">
        <div>
          <h2>Replay</h2>
          <p>{replay.frameCount} frames · {formatDuration(replay.cursorMs)} / {formatDuration(replay.durationMs)}</p>
        </div>
        <RotateCcw size={20} />
      </div>

      <div class="button-row">
        <button type="button" class="icon-button quiet" onclick={replay.playing ? pauseReplay : playReplay}>
          {#if replay.playing}
            <CirclePause size={18} />
            <span>Pause</span>
          {:else}
            <CirclePlay size={18} />
            <span>Play</span>
          {/if}
        </button>
        <label class="file-button">
          <Upload size={18} />
          <span>Load</span>
          <input type="file" accept=".sylg,application/vnd.synapse.log,application/octet-stream" onchange={loadReplay} />
        </label>
      </div>

      <input class="range" type="range" min="0" max={Math.max(1, replay.durationMs)} value={replay.cursorMs} oninput={seekReplay} />
      <label class="field inline">
        <span>Speed</span>
        <input type="range" min="0.25" max="4" step="0.25" value={replay.speed} oninput={setReplaySpeed} />
        <strong>{replay.speed.toFixed(2)}x</strong>
      </label>
    </section>

    <section class="panel recording-panel">
      <div class="panel-heading">
        <div>
          <h2>Logging</h2>
          <p>{recordingCount} frames</p>
        </div>
        <Download size={20} />
      </div>

      <div class="button-row">
        {#if recording}
          <button type="button" class="icon-button danger" onclick={stopRecording}>
            <Square size={18} />
            <span>Stop</span>
          </button>
        {:else}
          <button type="button" class="icon-button primary" onclick={startRecording}>
            <CirclePlay size={18} />
            <span>Record</span>
          </button>
        {/if}
        <button type="button" class="icon-button quiet" onclick={exportRecording}>
          <Download size={18} />
          <span>Export</span>
        </button>
      </div>
    </section>

    <section class="panel plot-panel">
      <div class="panel-heading">
        <div>
          <h2>Plots</h2>
          <p>{plotState.series.length} fields · {plotPackets.filter((packet) => packet.active).length} live packets</p>
        </div>
        <Activity size={20} />
      </div>

      <div class="plot-controls">
        {#each plotTraces as trace, index}
          <div class="plot-trace-control">
            <span class="plot-swatch" style={`background: ${plotTraceColors[index]};`}></span>
            <label>
              <span>Packet</span>
              <select
                value={trace.packetKey}
                onchange={(event) => updatePlotTrace(index, { packetKey: (event.currentTarget as HTMLSelectElement).value })}
              >
                {#each plotPackets as packet}
                  <option value={packet.key}>{packetOptionLabel(packet)}</option>
                {/each}
              </select>
            </label>
            <label>
              <span>Field</span>
              <select
                value={trace.fieldPath}
                onchange={(event) => updatePlotTrace(index, { fieldPath: (event.currentTarget as HTMLSelectElement).value })}
              >
                {#each fieldsForPacket(trace.packetKey) as field}
                  <option value={field.path}>{field.label}{field.units ? ` · ${field.units}` : ''}</option>
                {/each}
              </select>
            </label>
          </div>
        {/each}
      </div>

      <div class="plot-frame">
        <svg class="plot" viewBox="0 0 360 132" preserveAspectRatio="none" aria-label="Selected packet field plot">
          {#each [0, 1, 2, 3] as gridLine}
            <line class="plot-grid-line" x1="0" x2="360" y1={gridLine * 44} y2={gridLine * 44} />
          {/each}
          {#each resolvedPlotTraces as trace}
            {#if trace.path}
              <path class="plot-line" d={trace.path} style={`stroke: ${trace.color};`} />
            {/if}
          {/each}
        </svg>
        {#if !plotHasSamples}
          <div class="plot-empty">Waiting for samples</div>
        {/if}
      </div>

      <div class="plot-legend">
        {#each resolvedPlotTraces as trace}
          <div>
            <span class="plot-swatch" style={`background: ${trace.color};`}></span>
            <strong>{trace.packet?.label ?? 'Packet'} · {trace.field?.label ?? trace.fieldPath}</strong>
            <span>{formatPlotValue(trace.lastValue, trace.field?.units)} · {trace.rangeLabel}</span>
          </div>
        {/each}
      </div>
    </section>

    <section class="panel events-panel">
      <div class="panel-heading">
        <div>
          <h2>Events</h2>
          <p>{warningCount} warnings</p>
        </div>
        <AlertTriangle size={20} />
      </div>

      <div class="event-list">
        {#each vehicle.events.slice(0, 8) as event}
          <div class="event-row" class:warn={event.severity === 'warning'} class:danger={event.severity === 'error'}>
            <span>{event.severity}</span>
            <strong>{event.message}</strong>
          </div>
        {:else}
          <div class="empty-row">No events</div>
        {/each}
      </div>
    </section>

    <section class="panel topics-panel">
      <div class="panel-heading">
        <div>
          <h2>Topics</h2>
          <p>{topicRows.length} active · {Object.keys(TOPIC_DEFINITIONS).length} registered</p>
        </div>
        <Activity size={20} />
      </div>

      <div class="topic-table">
        <div class="topic-row header">
          <span>Topic</span>
          <span>Rate</span>
          <span>Latency</span>
          <span>State</span>
        </div>
        {#each topicRows as topic}
          <div class="topic-row">
            <span title={topic.topic}>{topic.label}</span>
            <span>{topic.rateHz.toFixed(1)} Hz</span>
            <span>{topic.latencyMs.toFixed(0)} ms</span>
            <span class:warn={topic.stale} class:ok={!topic.stale}>{topic.stale ? 'stale' : 'fresh'}</span>
          </div>
        {:else}
          <div class="empty-row">No topics</div>
        {/each}
      </div>
    </section>
  </div>
</main>

<style>
  :global(html) {
    background: #f6f7f8;
    color: #171b1f;
    font-family:
      Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    letter-spacing: 0;
  }

  :global(body) {
    margin: 0;
    min-width: 320px;
  }

  :global(*) {
    box-sizing: border-box;
  }

  .shell {
    min-height: 100vh;
    padding: 18px;
  }

  .topbar,
  .status-strip,
  .panel {
    border: 1px solid #d9dee3;
    background: #ffffff;
    border-radius: 8px;
  }

  .topbar {
    display: grid;
    grid-template-columns: 1fr auto auto;
    gap: 16px;
    align-items: center;
    padding: 14px;
  }

  .brand {
    display: flex;
    align-items: center;
    gap: 12px;
    min-width: 0;
  }

  .brand-mark {
    display: flex;
    width: 76px;
    height: 42px;
    align-items: center;
    justify-content: center;
    overflow: hidden;
    padding: 5px 6px;
    place-items: center;
    border-radius: 8px;
    background: #11161b;
  }

  .brand-mark img {
    display: block;
    width: 100%;
    height: 100%;
    object-fit: contain;
  }

  h1,
  h2,
  p {
    margin: 0;
  }

  h1 {
    font-size: 1.35rem;
    line-height: 1.1;
  }

  h2 {
    font-size: 1rem;
    line-height: 1.2;
  }

  p {
    color: #66717c;
    font-size: 0.86rem;
  }

  .mode-control {
    display: grid;
    grid-template-columns: repeat(4, minmax(78px, 1fr));
    min-height: 38px;
    overflow: hidden;
    border: 1px solid #cfd6dd;
    border-radius: 8px;
  }

  .mode-control label {
    display: grid;
    place-items: center;
    padding: 0 12px;
    color: #5c6873;
    cursor: pointer;
    font-size: 0.9rem;
    text-transform: capitalize;
  }

  .mode-control label + label {
    border-left: 1px solid #cfd6dd;
  }

  .mode-control input {
    position: absolute;
    opacity: 0;
    pointer-events: none;
  }

  .mode-control label.active {
    background: #152129;
    color: #ffffff;
  }

  .header-actions,
  .button-row {
    display: flex;
    align-items: center;
    gap: 8px;
    flex-wrap: wrap;
  }

  button,
  input,
  select,
  .file-button {
    font: inherit;
  }

  button,
  .file-button {
    border: 0;
    min-height: 38px;
    border-radius: 8px;
    cursor: pointer;
  }

  .icon-button,
  .file-button {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    padding: 0 12px;
    white-space: nowrap;
  }

  .icon-button.primary,
  .command-button.primary {
    background: #15836f;
    color: #ffffff;
  }

  .icon-button.quiet,
  .file-button,
  .command-button {
    background: #eef2f4;
    color: #263039;
  }

  .icon-button.danger,
  .command-button.danger {
    background: #b83e4b;
    color: #ffffff;
  }

  .status-strip {
    display: grid;
    grid-template-columns: repeat(4, minmax(120px, 1fr));
    gap: 10px;
    margin-top: 12px;
    padding: 10px;
  }

  .strip-item {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    min-height: 36px;
    border-radius: 8px;
    background: #f2f5f7;
    color: #58636e;
    text-transform: capitalize;
  }

  .ok {
    color: #147660;
  }

  .warn {
    color: #a36500;
  }

  .danger {
    color: #ab2f3f;
  }

  .dashboard {
    display: grid;
    grid-template-columns: minmax(260px, 0.85fr) minmax(340px, 1.4fr) minmax(300px, 1fr);
    gap: 12px;
    margin-top: 12px;
    align-items: start;
  }

  .panel {
    min-width: 0;
    padding: 14px;
  }

  .map-panel,
  .topics-panel,
  .events-panel {
    grid-row: span 2;
  }

  .panel-heading {
    display: flex;
    align-items: flex-start;
    justify-content: space-between;
    gap: 12px;
    margin-bottom: 12px;
  }

  .map-heading {
    gap: 10px;
  }

  .map-tools {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .map-view-control {
    display: grid;
    grid-template-columns: repeat(3, minmax(58px, 1fr));
    overflow: hidden;
    min-height: 32px;
    border: 1px solid #cdd5dc;
    border-radius: 8px;
    background: #f5f7f8;
  }

  .map-view-control button {
    min-height: 32px;
    padding: 0 9px;
    border-radius: 0;
    background: transparent;
    color: #53606a;
    font-size: 0.72rem;
    font-weight: 760;
    text-transform: uppercase;
  }

  .map-view-control button + button {
    border-left: 1px solid #cdd5dc;
  }

  .map-view-control button.active {
    background: #152129;
    color: #ffffff;
  }

  .field {
    display: grid;
    gap: 6px;
    color: #596570;
    font-size: 0.84rem;
  }

  .field.inline {
    grid-template-columns: auto 1fr auto;
    align-items: center;
    margin-top: 10px;
  }

  input,
  select {
    width: 100%;
    min-height: 38px;
    border: 1px solid #cfd6dd;
    border-radius: 8px;
    padding: 0 10px;
    background: #ffffff;
    color: #1d252c;
  }

  .metrics {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 8px;
  }

  .metrics.compact {
    margin-top: 10px;
  }

  .hud-metrics {
    margin-bottom: 10px;
  }

  .metric {
    display: grid;
    gap: 4px;
    min-height: 70px;
    padding: 10px;
    border: 1px solid #e0e5ea;
    border-radius: 8px;
    background: #fafbfc;
  }

  .metric span {
    color: #6b7680;
    font-size: 0.78rem;
  }

  .metric strong {
    align-self: end;
    overflow-wrap: anywhere;
    font-size: 1.18rem;
  }

  .map-canvas {
    position: relative;
    overflow: hidden;
    height: 350px;
    border-radius: 8px;
    border: 1px solid #cdd5dc;
    background:
      linear-gradient(90deg, rgba(31, 72, 61, 0.12) 1px, transparent 1px),
      linear-gradient(rgba(31, 72, 61, 0.12) 1px, transparent 1px),
      radial-gradient(circle at 24% 22%, rgba(68, 158, 129, 0.18), transparent 24%),
      linear-gradient(135deg, #edf4ef, #eef2f6 62%, #f8f9f6);
    background-size: 28px 28px, 28px 28px, auto, auto;
  }

  .mission-line {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
  }

  .mission-line path {
    fill: none;
    stroke: #2e6fba;
    stroke-dasharray: 4 5;
    stroke-linecap: round;
    stroke-width: 1.8;
  }

  .vehicle-marker {
    position: absolute;
    display: grid;
    width: 42px;
    height: 42px;
    place-items: center;
    border-radius: 50%;
    background: #11161b;
    color: #54d0b5;
    box-shadow: 0 8px 22px rgba(21, 33, 41, 0.24);
  }

  .map-axes-marker {
    width: 100%;
    height: 100%;
    overflow: visible;
  }

  .map-axes-marker line {
    stroke-linecap: round;
    stroke-width: 3.6;
  }

  .map-axes-marker circle {
    fill: #f9fffc;
    stroke: #11161b;
    stroke-width: 1.4;
  }

  .map-axis-forward {
    stroke: #f9fffc;
  }

  .map-axis-right {
    stroke: #61a8ff;
  }

  #map-axis-forward path {
    fill: #f9fffc;
  }

  #map-axis-right path {
    fill: #61a8ff;
  }

  .map-home,
  .map-scale {
    position: absolute;
    border-radius: 8px;
    background: rgba(255, 255, 255, 0.86);
    padding: 6px 8px;
    color: #47535f;
    font-size: 0.75rem;
    font-weight: 700;
  }

  .map-home {
    left: 16px;
    bottom: 16px;
  }

  .map-scale {
    right: 16px;
    bottom: 16px;
  }

  .command-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 8px;
    margin-top: 10px;
  }

  .command-button {
    min-height: 42px;
    padding: 0 10px;
  }

  .attitude-display {
    position: relative;
    overflow: hidden;
    display: grid;
    height: 260px;
    place-items: center;
    border-radius: 8px;
    border: 1px solid #d3dae0;
    background: #101518;
  }

  .battery-bar {
    overflow: hidden;
    height: 18px;
    border-radius: 8px;
    background: #e7ecef;
  }

  .battery-bar span {
    display: block;
    height: 100%;
    border-radius: inherit;
    background: linear-gradient(90deg, #15836f, #e0a127);
  }

  .range {
    margin-top: 12px;
  }

  .file-button {
    position: relative;
  }

  .file-button input {
    position: absolute;
    inset: 0;
    opacity: 0;
    cursor: pointer;
  }

  .plot {
    width: 100%;
    height: 128px;
    border: 1px solid #d8dee4;
    border-radius: 8px;
    background: linear-gradient(#ffffff, #f7fafb);
  }

  .plot-line {
    fill: none;
    stroke-linecap: round;
    stroke-width: 2.4;
  }

  .event-list,
  .topic-table {
    display: grid;
    gap: 6px;
  }

  .event-row,
  .topic-row,
  .empty-row {
    display: grid;
    align-items: center;
    min-height: 36px;
    border-radius: 8px;
    background: #f6f8fa;
    padding: 8px 10px;
    font-size: 0.86rem;
  }

  .event-row {
    grid-template-columns: 72px 1fr;
    gap: 8px;
  }

  .event-row span {
    text-transform: capitalize;
  }

  .topic-row {
    grid-template-columns: minmax(96px, 1fr) 76px 76px 60px;
    gap: 8px;
  }

  .topic-row.header {
    background: #ffffff;
    color: #697580;
    font-size: 0.75rem;
    font-weight: 700;
  }

  .topic-row span {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .empty-row {
    color: #697580;
  }

  @media (max-width: 1180px) {
    .dashboard {
      grid-template-columns: minmax(260px, 1fr) minmax(320px, 1fr);
    }

    .map-panel,
    .topics-panel,
    .events-panel {
      grid-row: auto;
    }
  }

  @media (max-width: 780px) {
    .shell {
      padding: 10px;
    }

    .topbar {
      grid-template-columns: 1fr;
    }

    .mode-control,
    .status-strip,
    .dashboard {
      grid-template-columns: 1fr;
    }

    .header-actions {
      justify-content: stretch;
    }

    .header-actions .icon-button,
    .button-row .icon-button,
    .file-button {
      flex: 1;
    }

    .map-canvas {
      height: 280px;
    }

    .topic-row {
      grid-template-columns: minmax(92px, 1fr) 68px 68px 56px;
      font-size: 0.8rem;
    }
  }

  /* Operations console refresh */
  :global(html) {
    background:
      linear-gradient(180deg, #0a0d0e 0%, #101314 52%, #080b0c 100%);
    color: #ecf4ef;
    font-family:
      Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    letter-spacing: 0;
  }

  :global(body) {
    background: #0a0d0e;
    overflow-x: hidden;
  }

  .shell {
    min-height: 100vh;
    padding: 12px;
    overflow-x: hidden;
    background:
      linear-gradient(90deg, rgba(255, 255, 255, 0.024) 1px, transparent 1px),
      linear-gradient(rgba(255, 255, 255, 0.018) 1px, transparent 1px),
      linear-gradient(180deg, #0b0f10, #111516 48%, #090c0d);
    background-size: 36px 36px, 36px 36px, auto;
  }

  .topbar,
  .status-strip,
  .panel {
    border: 1px solid #263239;
    border-radius: 8px;
    background: rgba(17, 23, 25, 0.92);
    box-shadow:
      inset 0 1px 0 rgba(255, 255, 255, 0.055),
      0 16px 42px rgba(0, 0, 0, 0.24);
  }

  .topbar {
    grid-template-columns: minmax(240px, 1fr) minmax(360px, auto) auto;
    min-height: 72px;
    padding: 10px 12px;
    background:
      linear-gradient(90deg, rgba(66, 232, 196, 0.08), transparent 28%),
      rgba(14, 20, 22, 0.96);
  }

  .brand {
    gap: 12px;
  }

  .brand-mark {
    width: 76px;
    height: 42px;
    border: 1px solid rgba(66, 232, 196, 0.44);
    border-radius: 8px;
    background: #050808;
    box-shadow: 0 0 28px rgba(66, 232, 196, 0.16);
  }

  h1 {
    color: #f4fbf7;
    font-size: 1.15rem;
    font-weight: 760;
  }

  h2 {
    color: #f4fbf7;
    font-size: 0.86rem;
    font-weight: 760;
    letter-spacing: 0.02em;
    text-transform: uppercase;
  }

  p {
    color: #84938d;
    font-size: 0.78rem;
  }

  .mode-control {
    grid-template-columns: repeat(4, minmax(82px, 1fr));
    min-height: 42px;
    border: 1px solid #2d3a41;
    border-radius: 8px;
    background: #0b1012;
  }

  .mode-control label {
    color: #8fa09a;
    font-size: 0.82rem;
    font-weight: 680;
    text-transform: uppercase;
  }

  .mode-control label + label {
    border-left: 1px solid #2d3a41;
  }

  .mode-control label.active {
    background: #dfe9e4;
    color: #0a1111;
  }

  .icon-button,
  .file-button,
  .command-button {
    min-height: 42px;
    border: 1px solid transparent;
    border-radius: 8px;
    font-size: 0.88rem;
    font-weight: 720;
    transition:
      background 140ms ease,
      border-color 140ms ease,
      color 140ms ease,
      transform 140ms ease;
  }

  .icon-button:hover,
  .file-button:hover,
  .command-button:hover {
    transform: translateY(-1px);
  }

  .icon-button.primary,
  .command-button.primary {
    border-color: rgba(66, 232, 196, 0.34);
    background: #0aa98d;
    color: #031211;
  }

  .icon-button.quiet,
  .file-button,
  .command-button {
    border-color: #2b383f;
    background: #172023;
    color: #d9e4df;
  }

  .icon-button.danger,
  .command-button.danger {
    border-color: rgba(255, 93, 115, 0.34);
    background: #c94158;
    color: #fff5f6;
  }

  .status-strip {
    grid-template-columns: repeat(4, minmax(150px, 1fr));
    gap: 8px;
    margin-top: 10px;
    padding: 8px;
    background: rgba(10, 15, 16, 0.92);
  }

  .strip-item {
    justify-content: flex-start;
    min-height: 44px;
    padding: 0 14px;
    border: 1px solid #243137;
    border-radius: 8px;
    background: #11191c;
    color: #a8b8b2;
    font-size: 0.88rem;
    font-weight: 720;
    text-transform: uppercase;
  }

  .strip-item :global(svg) {
    color: currentColor;
  }

  .ok {
    color: #42e8c4;
  }

  .warn {
    color: #ffbf57;
  }

  .danger {
    color: #ff6f82;
  }

  .dashboard {
    grid-template-columns: minmax(280px, 0.66fr) minmax(560px, 1.55fr) minmax(390px, 0.95fr);
    grid-template-areas:
      "connection vehicle map"
      "command attitude map"
      "replay plot events"
      "recording power topics";
    gap: 10px;
    margin-top: 10px;
  }

  .connection-panel {
    grid-area: connection;
  }

  .vehicle-panel {
    grid-area: vehicle;
  }

  .map-panel {
    grid-area: map;
  }

  .command-panel {
    grid-area: command;
  }

  .attitude-panel {
    grid-area: attitude;
  }

  .power-panel {
    grid-area: power;
  }

  .replay-panel {
    grid-area: replay;
  }

  .recording-panel {
    grid-area: recording;
  }

  .plot-panel {
    grid-area: plot;
  }

  .events-panel {
    grid-area: events;
  }

  .topics-panel {
    grid-area: topics;
  }

  .map-panel,
  .topics-panel,
  .events-panel {
    grid-row: auto;
  }

  .panel {
    position: relative;
    overflow: hidden;
    min-width: 0;
    padding: 12px;
  }

  .panel::before {
    position: absolute;
    inset: 0 0 auto;
    height: 2px;
    background: linear-gradient(90deg, #42e8c4, rgba(97, 168, 255, 0.7), transparent 76%);
    content: "";
    opacity: 0.62;
  }

  .panel-heading {
    align-items: center;
    margin-bottom: 10px;
  }

  .panel-heading > :global(svg) {
    color: #697c75;
  }

  .map-view-control {
    border-color: #2d3a41;
    background: #0b1012;
  }

  .map-view-control button {
    color: #8fa09a;
  }

  .map-view-control button + button {
    border-left-color: #2d3a41;
  }

  .map-view-control button.active {
    background: #dfe9e4;
    color: #0a1111;
  }

  .field {
    gap: 7px;
    color: #91a39c;
    font-size: 0.76rem;
    font-weight: 700;
    text-transform: uppercase;
  }

  .field.inline {
    min-width: 0;
  }

  .transport-readout {
    display: grid;
    gap: 8px;
    min-height: 70px;
    padding: 12px;
    border: 1px solid #27343a;
    border-radius: 8px;
    background: #0c1214;
  }

  .transport-readout span {
    color: #91a39c;
    font-size: 0.74rem;
    font-weight: 760;
    text-transform: uppercase;
  }

  .transport-readout strong {
    color: #dfe9e4;
    overflow-wrap: anywhere;
    font-size: 0.95rem;
  }

  .manual-service {
    display: grid;
    gap: 8px;
  }

  .manual-service > div {
    display: grid;
    gap: 6px;
    min-height: 56px;
    padding: 10px 12px;
    border: 1px solid #27343a;
    border-radius: 8px;
    background: #0c1214;
  }

  .manual-service span,
  .channel-strip span {
    color: #91a39c;
    font-size: 0.7rem;
    font-weight: 760;
    text-transform: uppercase;
  }

  .manual-service strong {
    color: #edf6f1;
    overflow-wrap: anywhere;
    font-size: 0.9rem;
  }

  .channel-strip {
    display: grid;
    grid-template-columns: repeat(5, minmax(0, 1fr));
    gap: 6px;
    margin-top: 8px;
  }

  .channel-strip > div {
    display: grid;
    gap: 4px;
    min-height: 48px;
    place-items: center;
    border: 1px solid #27343a;
    border-radius: 8px;
    background: #11191c;
  }

  .channel-strip strong {
    color: #42e8c4;
    font-size: 1rem;
  }

  input,
  select {
    min-height: 42px;
    border: 1px solid #314148;
    border-radius: 8px;
    background: #0b1113;
    color: #edf6f1;
    outline: none;
  }

  input:focus,
  select:focus {
    border-color: rgba(66, 232, 196, 0.72);
    box-shadow: 0 0 0 3px rgba(66, 232, 196, 0.13);
  }

  select {
    color-scheme: dark;
  }

  .metrics {
    gap: 8px;
  }

  .hud-metrics {
    grid-template-columns: repeat(2, minmax(0, 1fr));
    margin-bottom: 10px;
  }

  .metric {
    min-height: 82px;
    border: 1px solid #27343a;
    border-radius: 8px;
    background:
      linear-gradient(180deg, rgba(255, 255, 255, 0.035), transparent),
      #0e1517;
  }

  .metric span {
    color: #84938d;
    font-size: 0.72rem;
    font-weight: 760;
    text-transform: uppercase;
  }

  .metric strong {
    color: #f3fbf7;
    font-size: 1.42rem;
    font-weight: 780;
  }

  .map-canvas {
    height: 420px;
    border: 1px solid #2a383f;
    border-radius: 8px;
    background:
      linear-gradient(90deg, rgba(99, 178, 161, 0.12) 1px, transparent 1px),
      linear-gradient(rgba(99, 178, 161, 0.12) 1px, transparent 1px),
      linear-gradient(135deg, rgba(66, 232, 196, 0.18), transparent 34%),
      linear-gradient(155deg, #162124, #10181b 58%, #0d1315);
    background-size: 30px 30px, 30px 30px, auto, auto;
  }

  .map-panel .map-canvas,
  :global(.map-panel .indoor-scene) {
    height: calc(100% - 50px);
    min-height: 420px;
  }

  .mission-line path {
    stroke: #61a8ff;
    stroke-dasharray: 3 7;
    stroke-linecap: round;
    stroke-width: 2.2;
  }

  .vehicle-marker {
    width: 46px;
    height: 46px;
    border: 1px solid rgba(66, 232, 196, 0.46);
    border-radius: 50%;
    background: #050808;
    color: #42e8c4;
    box-shadow:
      0 0 0 8px rgba(66, 232, 196, 0.06),
      0 0 34px rgba(66, 232, 196, 0.22);
  }

  .map-axes-marker line {
    stroke-width: 3.8;
  }

  .map-axes-marker circle {
    fill: #edf6f1;
    stroke: #050808;
  }

  .map-axis-forward {
    stroke: #edf6f1;
  }

  .map-axis-right {
    stroke: #61a8ff;
  }

  #map-axis-forward path {
    fill: #edf6f1;
  }

  #map-axis-right path {
    fill: #61a8ff;
  }

  .map-home,
  .map-scale {
    border: 1px solid #2a383f;
    border-radius: 8px;
    background: rgba(9, 14, 15, 0.82);
    color: #dfe9e4;
    font-size: 0.72rem;
  }

  .command-grid {
    gap: 8px;
  }

  .command-button {
    min-height: 48px;
  }

  .attitude-display {
    min-height: 320px;
    height: calc(100% - 178px);
    padding: 10px;
    border: 1px solid #2a383f;
    border-radius: 8px;
    background: linear-gradient(180deg, #10191b, #0a0f10);
  }

  .attitude-instrument {
    width: min(100%, 330px);
    height: min(100%, 330px);
    max-height: 330px;
    filter: drop-shadow(0 22px 38px rgba(0, 0, 0, 0.45));
  }

  .instrument-plate {
    fill: #293332;
    stroke: #53615c;
    stroke-opacity: 0.7;
    stroke-width: 1.4;
  }

  .mount-screw {
    fill: #152022;
    stroke: #94a29d;
    stroke-opacity: 0.62;
    stroke-width: 1.5;
  }

  .bezel-outer {
    fill: #050708;
    stroke: #161f22;
    stroke-width: 18;
  }

  .bezel-inner {
    fill: #0b0f10;
    stroke: #202b2f;
    stroke-width: 8;
  }

  .sky {
    fill: #79d7f5;
  }

  .ground {
    fill: #9a6732;
  }

  .horizon-line {
    stroke: #f7faf7;
    stroke-width: 2.2;
  }

  .pitch-mark line {
    stroke: rgba(255, 255, 255, 0.78);
    stroke-linecap: round;
    stroke-width: 1.9;
  }

  .pitch-mark text {
    fill: rgba(255, 255, 255, 0.84);
    font-size: 10px;
    font-weight: 760;
    text-anchor: middle;
  }

  .face-shadow {
    fill: none;
    stroke: rgba(0, 0, 0, 0.48);
    stroke-width: 3;
  }

  .roll-scale line {
    stroke: #f5faf7;
    stroke-linecap: round;
    stroke-width: 2.6;
  }

  .roll-scale line.major {
    stroke-width: 4;
  }

  .top-index {
    fill: #dff7ff;
    stroke: #0d2025;
    stroke-width: 1.4;
  }

  .bank-pointer path {
    fill: #e9fff9;
    stroke: #071314;
    stroke-linejoin: round;
    stroke-width: 1.5;
  }

  .bank-pointer line {
    stroke: #42e8c4;
    stroke-linecap: round;
    stroke-width: 2.2;
  }

  .aircraft-reference path {
    fill: none;
    stroke: #fff8e8;
    stroke-linecap: round;
    stroke-linejoin: round;
    stroke-width: 4.5;
  }

  .aircraft-reference circle {
    fill: #fff8e8;
    stroke: #19120b;
    stroke-width: 1.3;
  }

  .glass {
    fill: url("#attitude-glass");
    stroke: rgba(255, 255, 255, 0.05);
    stroke-width: 1;
    pointer-events: none;
  }

  .battery-bar {
    height: 20px;
    border: 1px solid #26343a;
    border-radius: 8px;
    background: #0a1011;
  }

  .battery-bar span {
    background: linear-gradient(90deg, #42e8c4, #ffbf57);
  }

  .range,
  input[type="range"] {
    min-width: 0;
    width: 100%;
    accent-color: #42e8c4;
  }

  .plot {
    height: 154px;
    border: 1px solid #2a383f;
    border-radius: 8px;
    background:
      linear-gradient(90deg, rgba(255, 255, 255, 0.045) 1px, transparent 1px),
      linear-gradient(rgba(255, 255, 255, 0.035) 1px, transparent 1px),
      #0b1113;
    background-size: 32px 32px, 32px 32px, auto;
  }

  .plot-line {
    stroke-width: 2.6;
  }

  .plot-controls {
    display: grid;
    gap: 8px;
    margin-bottom: 10px;
  }

  .plot-trace-control {
    display: grid;
    grid-template-columns: 10px minmax(0, 1fr) minmax(0, 0.9fr);
    gap: 8px;
    align-items: end;
  }

  .plot-trace-control label {
    display: grid;
    gap: 4px;
    min-width: 0;
  }

  .plot-trace-control label span {
    overflow: hidden;
    color: #778982;
    font-size: 0.64rem;
    font-weight: 760;
    text-overflow: ellipsis;
    text-transform: uppercase;
    white-space: nowrap;
  }

  .plot-trace-control select {
    min-height: 34px;
    padding: 0 8px;
    font-size: 0.76rem;
    font-weight: 680;
  }

  .plot-swatch {
    display: block;
    width: 10px;
    height: 10px;
    align-self: center;
    border-radius: 999px;
    box-shadow: 0 0 14px currentColor;
  }

  .plot-frame {
    position: relative;
    overflow: hidden;
    border-radius: 8px;
  }

  .plot {
    height: 132px;
  }

  .plot-grid-line {
    stroke: rgba(255, 255, 255, 0.08);
    stroke-width: 1;
  }

  .plot-empty {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    color: #778982;
    font-size: 0.78rem;
    font-weight: 700;
    pointer-events: none;
  }

  .plot-legend {
    display: grid;
    gap: 6px;
    margin-top: 10px;
  }

  .plot-legend > div {
    display: grid;
    grid-template-columns: 10px minmax(0, 1fr) auto;
    gap: 8px;
    align-items: center;
    min-width: 0;
    color: #91a39c;
    font-size: 0.72rem;
  }

  .plot-legend strong {
    overflow: hidden;
    color: #edf6f1;
    font-size: 0.78rem;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .plot-legend span:last-child {
    overflow: hidden;
    text-align: right;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .event-list,
  .topic-table {
    gap: 4px;
  }

  .event-row,
  .topic-row,
  .empty-row {
    min-height: 34px;
    border: 1px solid #233035;
    border-radius: 8px;
    background: #0c1214;
    color: #d7e2dd;
    font-size: 0.78rem;
  }

  .event-row strong {
    font-weight: 680;
  }

  .topic-row {
    grid-template-columns: minmax(96px, 1fr) 72px 72px 60px;
  }

  .topic-row.header {
    border-color: transparent;
    background: transparent;
    color: #778982;
    font-size: 0.68rem;
    text-transform: uppercase;
  }

  .empty-row {
    color: #778982;
  }

  @media (max-width: 1240px) {
    .dashboard {
      grid-template-columns: minmax(280px, 0.9fr) minmax(360px, 1.1fr);
      grid-template-areas:
        "connection vehicle"
        "command map"
        "attitude map"
        "replay events"
        "recording topics"
        "power plot";
    }

    .map-canvas {
      height: 380px;
    }
  }

  @media (max-width: 820px) {
    .shell {
      padding: 8px;
    }

    .topbar,
    .dashboard,
    .status-strip {
      grid-template-columns: 1fr;
    }

    .topbar {
      gap: 10px;
    }

    .mode-control {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }

    .mode-control label:nth-child(3) {
      border-left: 0;
      border-top: 1px solid #2d3a41;
    }

    .mode-control label:nth-child(4) {
      border-top: 1px solid #2d3a41;
    }

    .status-strip {
      gap: 6px;
    }

    .dashboard {
      grid-template-areas:
        "vehicle"
        "map"
        "command"
        "connection"
        "attitude"
        "power"
        "replay"
        "recording"
        "plot"
        "events"
        "topics";
    }

    .map-canvas {
      height: 300px;
    }

    .metrics,
    .command-grid {
      grid-template-columns: 1fr 1fr;
    }

    .topic-row {
      grid-template-columns: minmax(82px, 1fr) 62px 62px 52px;
      font-size: 0.74rem;
    }
  }

  .dashboard {
    grid-template-areas: none;
    grid-auto-flow: dense;
    grid-auto-rows: 8px;
    gap: 0 10px;
    align-items: stretch;
  }

  .panel {
    height: calc(100% - 10px);
  }

  .connection-panel {
    grid-area: auto;
    grid-column: 1;
    grid-row: span 22;
  }

  .command-panel {
    grid-area: auto;
    grid-column: 1;
    grid-row: span 47;
  }

  .manual-panel {
    grid-area: auto;
    grid-column: 1;
    grid-row: span 29;
  }

  .replay-panel {
    grid-area: auto;
    grid-column: 1;
    grid-row: span 27;
  }

  .recording-panel {
    grid-area: auto;
    grid-column: 1;
    grid-row: span 14;
  }

  .vehicle-panel {
    grid-area: auto;
    display: none;
    grid-column: 3;
    grid-row: span 30;
  }

  .attitude-panel {
    grid-area: auto;
    grid-column: 3;
    grid-row: span 68;
  }

  .power-panel {
    grid-area: auto;
    grid-column: 3;
    grid-row: span 25;
  }

  .plot-panel {
    grid-area: auto;
    grid-column: 2;
    grid-row: span 48;
  }

  .map-panel {
    grid-area: auto;
    grid-column: 2;
    grid-row: span 70;
  }

  .events-panel {
    grid-area: auto;
    grid-column: 3;
    grid-row: span 18;
  }

  .topics-panel {
    grid-area: auto;
    grid-column: 3;
    grid-row: span 35;
  }

  .events-panel .event-list,
  .topics-panel .topic-table {
    max-height: calc(100% - 42px);
    overflow: auto;
    padding-right: 2px;
  }

  @media (max-width: 1240px) {
    .dashboard {
      grid-template-columns: minmax(280px, 0.85fr) minmax(380px, 1.15fr);
      grid-auto-rows: 8px;
      gap: 0 10px;
    }

    .connection-panel,
    .command-panel,
    .manual-panel,
    .replay-panel,
    .recording-panel {
      grid-column: 1;
    }

    .vehicle-panel,
    .attitude-panel,
    .power-panel,
    .plot-panel,
    .map-panel,
    .events-panel,
    .topics-panel {
      grid-column: 2;
    }

    .map-panel {
      grid-row: span 55;
    }

    .plot-panel {
      grid-row: span 49;
    }
  }

  @media (max-width: 820px) {
    .dashboard {
      display: grid;
      grid-template-columns: 1fr;
      grid-auto-rows: auto;
    }

    .connection-panel,
    .command-panel,
    .manual-panel,
    .replay-panel,
    .recording-panel,
    .vehicle-panel,
    .attitude-panel,
    .power-panel,
    .plot-panel,
    .map-panel,
    .events-panel,
    .topics-panel {
      grid-column: 1;
      grid-row: auto;
      height: auto;
    }

    .events-panel .event-list,
    .topics-panel .topic-table {
      max-height: none;
      overflow: visible;
    }

    .map-heading {
      display: grid;
      grid-template-columns: 1fr;
    }

    .map-tools {
      width: 100%;
      justify-content: space-between;
    }

    .map-view-control {
      flex: 1;
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }
  }
</style>
