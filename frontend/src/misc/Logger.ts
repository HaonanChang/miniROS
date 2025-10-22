import chalk from "chalk";

enum MsgLevel {
	error,
	warning,
	info,
	verbose,
	debug,
}

export default class Logger {
	private static getProperMsg(msg: string | Error) {
		const isError = msg instanceof Error;
		return isError
			? `${(msg as Error).message} \n\tStack: ${(msg as Error).stack}`
			: msg.toString();
	}

	private static getAppName() {
		return import.meta.env.VITE_APP_NAME ?? "rdc-web";
	}

	private static print(
		msg: string | Error,
		msgLevel?: MsgLevel,
		colorFunction: (text: string) => string = (text) => text,
	): void {
		const isError = msg instanceof Error;

		if (msgLevel === undefined && isError) {
			msgLevel = MsgLevel.error;
		} else {
			msgLevel = msgLevel ?? MsgLevel.info;
		}

		const time = new Date().toISOString();

		const textPayload =
			[
				`${time}`,
				this.getAppName(),
				MsgLevel[msgLevel].toLocaleString().toUpperCase(),
				Logger.getProperMsg(msg),
			].join(" | ") + "\n";

		const payload = colorFunction(textPayload);

		console.log(payload);
	}

	private static make_message(...msgs: (string | Error)[]) {
		return msgs.map((msg) => Logger.getProperMsg(msg)).join(" | ");
	}

	static error(...msgs: (string | Error)[]) {
		Logger.print(Logger.make_message(...msgs), MsgLevel.error, chalk.red);
	}

	static debug(...msgs: (string | Error)[]) {
		Logger.print(Logger.make_message(...msgs), MsgLevel.debug);
	}

	static warning(...msgs: (string | Error)[]) {
		Logger.print(
			Logger.make_message(...msgs),
			MsgLevel.warning,
			chalk.yellow,
		);
	}

	static info(...msgs: (string | Error)[]) {
		Logger.print(Logger.make_message(...msgs), MsgLevel.info, chalk.blue);
	}

	static verbose(...msgs: (string | Error)[]) {
		Logger.print(Logger.make_message(...msgs), MsgLevel.verbose);
	}
}
