export default class KeyListener {
	private _keyDownState = new Map<string, boolean>();
	private _keyPressListeners = new Map<
		string,
		Set<(key: string) => Promise<void>>
	>();
	private _keyLiftListeners = new Map<
		string,
		Set<(key: string) => Promise<void>>
	>();

	private static _instance: KeyListener;

	private _onKeyDown(evt: KeyboardEvent) {
		console.log("Key down", evt.key);
		if (
			!this._keyDownState.has(evt.key) ||
			!this._keyDownState.get(evt.key)
		) {
			this._keyPressListeners.get(evt.key)?.forEach(async (listener) => {
				// TODO: Async wait here?
				listener(evt.key);
			});
		}

		this._keyDownState.set(evt.key, true);
	}

	private _onKeyUp(evt: KeyboardEvent) {
		console.log("Key up", evt.key);

		if (this._keyDownState === undefined) {
			this.init_vars();
		}

		if (
			this._keyDownState.has(evt.key) &&
			this._keyDownState.get(evt.key)
		) {
			this._keyLiftListeners.get(evt.key)?.forEach(async (listener) => {
				// TODO: Async wait here?
				listener(evt.key);
			});
		}

		this._keyDownState.set(evt.key, false);
	}

	public removeKeyPressListener(
		key: string,
		listener: (key: string) => Promise<void>,
	) {
		this._keyPressListeners.get(key)?.delete(listener);
	}

	public registerKeyPressListener(
		key: string,
		listener: (key: string) => Promise<void>,
	) {
		if (!this._keyPressListeners.has(key)) {
			this._keyPressListeners.set(key, new Set());
		}

		this._keyPressListeners.get(key)?.add(listener);
	}

	public registerKeyLiftListener(
		key: string,
		listener: (key: string) => Promise<void>,
	) {
		if (!this._keyLiftListeners.has(key)) {
			this._keyLiftListeners.set(key, new Set());
		}

		this._keyLiftListeners.get(key)?.add(listener);
	}

	private init_vars() {
		this._keyDownState = new Map<string, boolean>();
		this._keyPressListeners = new Map<
			string,
			Set<(key: string) => Promise<void>>
		>();
		this._keyLiftListeners = new Map<
			string,
			Set<(key: string) => Promise<void>>
		>();
	}

	public static getInstance() {
		if (!this._instance) {
			this._instance = new KeyListener();
			this._instance.init_vars();
		}
		return this._instance;
	}

	public static init() {
		// Avoid double registration
		window.removeEventListener("keydown", (evt) =>
			KeyListener.getInstance()._onKeyDown(evt),
		);
		window.removeEventListener("keyup", (evt) =>
			KeyListener.getInstance()._onKeyUp(evt),
		);

		window.addEventListener("keydown", (evt) =>
			KeyListener.getInstance()._onKeyDown(evt),
		);
		window.addEventListener("keyup", (evt) =>
			KeyListener.getInstance()._onKeyUp(evt),
		);

		// window.addEventListener("keydown", (event) => {
		//     console.log(event.key);
		// });

		// window.addEventListener("keyup", (event) => {
		//     console.log(event.key);
		// });
	}
}
