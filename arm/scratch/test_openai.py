from openai import OpenAI

client = OpenAI(
  base_url="https://zenmux.ai/api/v1",
  api_key="sk-ai-v1-054562863219e9c2c06bf7d395bce71c92d4b70eea40ecb1ca66f3bf05bda52e",
)

# Chat Completion
completion = client.chat.completions.create(
  model="anthropic/claude-fable-5-free",
  messages=[
    {
      "role": "user",
      "content": "What is the meaning of life?"
    }
  ]
)
print(completion.choices[0].message.content)

try:
    # Responses API (note: OpenAI SDK might not have responses.create natively unless using beta)
    responses = client.responses.create(
      model="anthropic/claude-fable-5-free",
      input="What is the meaning of life?"
    )
    print(responses)
except Exception as e:
    print(f"Error with responses API: {e}")
