import type { CommandIntent, ConnectionState, GcsFrame } from './types';

export type TransportMessage =
  | GcsFrame
  | {
      kind: 'commandAck';
      commandId: string;
      command: string;
      status: 'acked' | 'published' | 'rejected' | 'timeout';
      reason: string;
      sequence: number;
      receivedAtMs: number;
    };

export class WebSocketBridgeTransport {
  #socket: WebSocket | null = null;

  constructor(
    private readonly url: string,
    private readonly onMessage: (message: TransportMessage) => void,
    private readonly onConnection: (state: ConnectionState) => void
  ) {}

  connect(): void {
    this.onConnection({ mode: 'bridge', status: 'connecting', url: this.url, message: 'connecting' });
    this.#socket = new WebSocket(this.url);

    this.#socket.addEventListener('open', () => {
      this.onConnection({ mode: 'bridge', status: 'connected', url: this.url, message: 'connected' });
    });

    this.#socket.addEventListener('message', (event) => {
      if (typeof event.data !== 'string') {
        return;
      }

      this.onMessage(JSON.parse(event.data) as TransportMessage);
    });

    this.#socket.addEventListener('close', () => {
      this.onConnection({ mode: 'bridge', status: 'disconnected', url: this.url, message: 'closed' });
    });

    this.#socket.addEventListener('error', () => {
      this.onConnection({ mode: 'bridge', status: 'error', url: this.url, message: 'socket error' });
    });
  }

  sendCommand(command: CommandIntent): void {
    if (this.#socket?.readyState !== WebSocket.OPEN) {
      throw new Error('bridge socket is not open');
    }

    this.#socket.send(JSON.stringify(command));
  }

  disconnect(): void {
    this.#socket?.close();
    this.#socket = null;
  }
}

type ZenohWasmModule = typeof import('@cognipilot/zenoh-wasm');
type ZenohSession = import('@cognipilot/zenoh-wasm').ZenohSession;

const ZENOH_CONNECT_TIMEOUT_MS = 8_000;
const ZENOH_PUBLISH_TIMEOUT_MS = 2_000;

export class ZenohWasmTransport {
  #session: ZenohSession | null = null;
  #zenoh: ZenohWasmModule | null = null;
  #version = 'unknown';

  constructor(
    private readonly endpointOrConfig: string,
    private readonly onConnection: (state: ConnectionState) => void
  ) {}

  async connect(): Promise<void> {
    this.onConnection({
      mode: 'zenoh',
      status: 'connecting',
      url: this.endpointOrConfig,
      message: 'loading zenoh-wasm'
    });

    try {
      this.#zenoh = await import('@cognipilot/zenoh-wasm');
      await this.#zenoh.default();
      this.#zenoh.initPanicHook();
      this.#version = this.#zenoh.version();

      const input = this.endpointOrConfig.trim();
      this.#session = await withTimeout(
        input.startsWith('{') ? this.#zenoh.openWithConfig(input) : this.#zenoh.open(input),
        ZENOH_CONNECT_TIMEOUT_MS,
        `opening Zenoh endpoint ${this.endpointOrConfig}`
      );

      this.onConnection({
        mode: 'zenoh',
        status: 'connected',
        url: this.endpointOrConfig,
        message: `zenoh-wasm ${this.#version} connected`
      });
    } catch (error) {
      this.onConnection({
        mode: 'zenoh',
        status: 'error',
        url: this.endpointOrConfig,
        message: error instanceof Error ? error.message : String(error)
      });
      throw error;
    }
  }

  async sendCommand(command: CommandIntent): Promise<void> {
    if (!this.#session || this.#session.isClosed()) {
      throw new Error('zenoh-wasm session is not open');
    }

    await withTimeout(
      this.#session.putString(command.topic, JSON.stringify(command)),
      ZENOH_PUBLISH_TIMEOUT_MS,
      `publishing ${command.command} to ${command.topic}`
    );
  }

  async disconnect(): Promise<void> {
    const session = this.#session;
    this.#session = null;

    if (session && !session.isClosed()) {
      await session.close();
    }

    this.onConnection({
      mode: 'zenoh',
      status: 'disconnected',
      url: this.endpointOrConfig,
      message: 'zenoh-wasm session closed'
    });
  }

  get version(): string {
    return this.#version;
  }
}

async function withTimeout<T>(promise: Promise<T>, timeoutMs: number, description: string): Promise<T> {
  let timer: ReturnType<typeof setTimeout> | undefined;
  const timeout = new Promise<never>((_, reject) => {
    timer = setTimeout(() => reject(new Error(`Timed out while ${description}`)), timeoutMs);
  });

  try {
    return await Promise.race([promise, timeout]);
  } finally {
    if (timer) {
      clearTimeout(timer);
    }
  }
}
