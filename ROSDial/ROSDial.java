package rosdial;

import java.util.*;

import opendial.*;
import opendial.bn.distribs.IndependentProbDistribution;
import opendial.datastructs.Assignment;
import opendial.gui.GUIFrame;
import opendial.gui.TextOnlyInterface;
import opendial.modules.simulation.Simulator;
import opendial.plugins.MaltParser;
import opendial.readers.XMLDomainReader;
import opendial.readers.XMLSettingsReader;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;


public class ROSDial extends AbstractNodeMain {
	public GraphName getDefaultNodeName() {
		return GraphName.of("rosjava/node");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		final Log log = connectedNode.getLog();
		DialogueSystem system = new DialogueSystem();
		startUpSystem(system, log);
		Properties to_settings = new Properties();
		to_settings.put("parsingmodel", "/home/masx/de-graaf/engmalt.linear-1.7.mco");
		to_settings.put("taggingmodel", "/home/masx/de-graaf/english-caseless-left3words-distsim.tagger");
		system.getSettings().fillSettings(to_settings);
		system.changeDomain(XMLDomainReader.extractDomain("/home/masx/de-graaf/domain.xml"));
		MaltParser parser = new MaltParser(system);
		system.attachModule(parser);
		parser.start();
		Random randomNumber = new Random();

		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("DMsubscriber", std_msgs.String._TYPE);
		final Publisher<std_msgs.String> publisher = connectedNode.newPublisher("DMpublisher", std_msgs.String._TYPE);

		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				String[] toAsk = message.getData().split("~")[1].split(":");
				String inputMessage = message.getData().split("~")[0];
				String[] data = inputMessage.split(":");
				String toAddToSystem = "";
				log.info("I heard: \"" + inputMessage + "\" and \"" + toAsk + "\"");
				if (data[0].equals("input")){
					system.addUserInput(data[1]);
				}
				else if (data[0].equals("detected")){
					int numberOfObjects = Integer.parseInt(data[1]);
					if (numberOfObjects == 0)
						toAddToSystem = "I don't know what this is.";
					if (numberOfObjects == 1)
						toAddToSystem = "The object is thought to be " + data[3].split(",")[0];
					else if (numberOfObjects == 2 || !data[2].equals("")) {
						if (data[2].equals("unknown")){
							toAddToSystem = "One of the objects is " + data[3].split(",")[0] + 
									", the other is " + data[4].split(",")[0] + 
									". Please tell me where these things are";
						}
						else if (data[2].equals("same")){
							toAddToSystem = "I think this is " + data[3].split(",")[0] + 
									" , but maybe it is " + data[4].split(",")[0];
						}
						else {
							toAddToSystem = "The " + data[3].split(",")[0] + " is " + data[2] + " the "+ data[4].split(",")[0];
						}
					}
					else if (numberOfObjects > 2) {
						toAddToSystem = "The object is thought to be " + data[3].split(",")[0] + 
								", but there are a couple of other candidates including " + data[4].split(",")[0] + 
								" and " + data[5].split(",")[0];
					}
				}

				else if (data[0].equals("learned"))
					toAddToSystem = "Ok, I learned " + data[1];
				else if (data[0].equals("reinforced"))
					toAddToSystem = "Ok, this is " + data[1];
				else if (data[0].equals("detected none"))
					toAddToSystem = "I don't know what this is";
				else if (data[0].equals("unlearned"))
					toAddToSystem = "Okay, sorry";
				else if (data[0].equals("removed"))
					toAddToSystem = "Okay, I forgot about it";
				else if (data[0].equals("changed"))
					toAddToSystem = "Okay, changed " + data[1] + " to " + data[2];
				else if (data[0].equals("relation"))
					toAddToSystem = "Okay, this relation is called " + data[1];
				else if (data[0].equals("get")){
					IndependentProbDistribution varContents = system.getContent(data[1]); 
					std_msgs.String str = publisher.newMessage();
					str.setData(varContents.toString());
					publisher.publish(str);
				}
				else if (data[0].equals("list")){
					std_msgs.String str = publisher.newMessage();
					str.setData(system.getState().toString());
					publisher.publish(str);
				}
				else if(data[0].equals("domain")){
					system.changeDomain(XMLDomainReader.extractDomain(data[1]));
				}
				else if(data[0].equals("failed recognizing objects"))
					toAddToSystem = "I did not recognize both of the objects, so I can not learn this";
				else
					toAddToSystem = "I did not understand that";
				System.out.println(toAsk);
				if (toAsk[0].equals("relation")){
					int generatedNumber = randomNumber.nextInt(4);
					if (generatedNumber == 0)
						toAddToSystem += ". Could you show me more relations?";
					else if (generatedNumber == 1)
						toAddToSystem += ". I am curious where things are.";
				}
				else if(toAsk[0].equals("object")){
					int generatedNumber = randomNumber.nextInt(4);
					if (generatedNumber == 0)
						toAddToSystem += ". Could you show me more of " + toAsk[1] + "?";
					else if (generatedNumber == 1)
						toAddToSystem += ". I would like to see more of " + toAsk[1] + ".";
					else
						toAddToSystem += ".";
					}
				else
					toAddToSystem += ".";
				addSystemOutput(system, toAddToSystem);
			}
		});
		if (!system.getDomain().toString().equals("")){
			connectedNode.executeCancellableLoop(new CancellableLoop() {
				@Override
				protected void loop() throws InterruptedException {
					if (!system.getContent("to_do").getValues().iterator().next().toString().equals("")){
						std_msgs.String str = publisher.newMessage();
						if (system.getContent("to_do").getValues().iterator().next().toString().equals("learn_object")){
							str.setData("learn:" + system.getContent("shown_object").getValues().iterator().next());
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("what_is_object")){
							str.setData("what is this?");
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("reinforce_object")){
							str.setData("last-object");
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("unlearn_last_object")){
							str.setData("unlearn-last-object");
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("unlearn_object")){
							str.setData("unlearn-object:" + system.getContent("shown_object").getValues().iterator().next());
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("reinforce_relation")){
							str.setData("last-relation");
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("unlearn_last_relation")){
							str.setData("unlearn-last-relation");
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("unlearn_relation")){
							str.setData("unlearn-relation:" + system.getContent("shown_relation").getValues().iterator().next());
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("relearn")){
							str.setData("relearn:" + system.getContent("shown_object").getValues().iterator().next());
						}
						else if (system.getContent("to_do").getValues().iterator().next().toString().equals("learn_relation")){
							str.setData("relation:" + 
									system.getContent("shown_object1").getValues().iterator().next() + ":" + 
									system.getContent("shown_object2").getValues().iterator().next() + ":" + 
									system.getContent("shown_relation").getValues().iterator().next());
						}
						else {
							;
							log.info(system.getContent("to_do").getValues().iterator().next());
						}
						addToDo(system, "");
						publisher.publish(str);
					}
					Thread.sleep(100);
				}
			});
		}
		else {
			log.info(system.getDomain());
			log.info("set a domain");
		}

	}
	public Set<String> addSystemOutput(DialogueSystem system, String systemOutput) throws RuntimeException {
		// Opendial has addUserInput. addSystemOutput is just a shortcut for doing the same for the system.
		Assignment a = new Assignment("u_m", systemOutput);
		return system.addContent(a);
	}

	public Set<String> addToDo(DialogueSystem system, String toDo) throws RuntimeException {
		// Sets a new entry for the to_do variable, and keeps a spare of the old one in last_step.
		Assignment a = new Assignment("to_do", toDo);
		Assignment b = new Assignment("last_step", system.getContent("to_do").sample());
		system.addContent(b);
		return system.addContent(a);
	}

	public void startUpSystem(DialogueSystem system, Log log) {
		// Practically copied from the Opendial files. This starts up the opendial system.
		try {
			String domainFile = System.getProperty("domain");
			String settingsFile = System.getProperty("settings");
			String dialogueFile = System.getProperty("dialogue");
			String simulatorFile = System.getProperty("simulator");

			system.getSettings().fillSettings(System.getProperties());
			if (domainFile != null) {
				system.changeDomain(XMLDomainReader.extractDomain(domainFile));
				log.info("Domain from " + domainFile + " successfully extracted");
			}
			if (settingsFile != null) {
				system.getSettings().fillSettings(XMLSettingsReader.extractMapping(settingsFile));
				log.info("Settings from " + settingsFile + " successfully extracted");		
			}
			if (dialogueFile != null) {
				system.importDialogue(dialogueFile);
			}
			if (simulatorFile != null) {
				Simulator simulator = new Simulator(system, XMLDomainReader.extractDomain(simulatorFile));
				log.info("Simulator with domain " + simulatorFile + " successfully extracted");		
				system.attachModule(simulator);
			}
			Settings settings = system.getSettings();
			system.changeSettings(settings);
			if (!settings.showGUI) {
				system.attachModule(new TextOnlyInterface(system));
			}

			if (!settings.remoteConnections.isEmpty() && settings.showGUI) {
				system.getModule(GUIFrame.class).enableSpeech(true);
			}
			system.startSystem();
			log.info("Dialogue system started!");
		}
		catch (RuntimeException e) {
			log.info("could not start system, aborting: " + e);
		}

	}
}
